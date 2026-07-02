/**
 * Copyright (C) 2017, Raffaello Bonghi <raffaello@rnext.it>
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "roboteq/serial_controller.h"

#include <algorithm>
#include <boost/algorithm/string.hpp>

namespace roboteq {

const std::string eol("\r");
const size_t max_line_length(128);

serial_controller::serial_controller(string port, unsigned long baudrate, uint32_t serial_timeout_ms,
                                     uint32_t response_timeout_ms, uint32_t max_retries)
    : mSerialPort(port)
    , mBaudrate(baudrate)
    , mTimeout(serial_timeout_ms)
    , mResponseTimeoutMs(response_timeout_ms)
    , mMaxRetries(std::max<uint32_t>(1, max_retries))
    , mCommandData(false)
    , mQueryData(false)
{
    // Keep timeout values in a valid range.
    if (mTimeout == 0) {
        mTimeout = 1;
    }
    if (mResponseTimeoutMs == 0) {
        mResponseTimeoutMs = 1;
    }
}

serial_controller::~serial_controller()
{
    stop();
}

bool serial_controller::start()
{
    try
    {
        mSerial.setPort(mSerialPort);
        mSerial.open();
        mSerial.setBaudrate(mBaudrate);

        serial::Timeout to = serial::Timeout::simpleTimeout(mTimeout);
        mSerial.setTimeout(to);
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open serial port " << mSerialPort << " - Error: "  << e.what() );
        return false;
    }

    if(mSerial.isOpen()){
        ROS_DEBUG_STREAM("Serial Port correctly initialized: " << mSerialPort );
    }
    else
    {
        ROS_ERROR_STREAM( "Serial port not opened: " << mSerialPort );
        return false;
    }
    // Initialize stop function
    mStopping = false;
    // Launch async reader thread
    first = std::thread(&serial_controller::async_reader, this);
    ROS_DEBUG_STREAM( "Serial port ready" );

   
    // Launch script and check version
    script(true);
    if(query("VAR", "1"))
    {
        _script_ver = get();
        // Stop script
        script(false);

        if(_script_ver.compare(_script_ver) != 0)
        {
            ROS_WARN_STREAM("Script version mismatch. Updating V"  << _script_ver << "->V"<< _script_ver << " ...");
            if(downloadScript())
            {
                ROS_WARN_STREAM("...Done!");
            }
            else
            {
                ROS_ERROR_STREAM("...ERROR to download the script!");
                return false;
            }
        } else
        {
            ROS_DEBUG_STREAM("Script V" << _script_ver);
        }
    }
   
    return true;
}

bool serial_controller::stop()
{
    // Stop script
    script(false);
    // Stop the reader
    mStopping = true;
    // Close the serial port
    mSerial.close();
    // Wait stop thread
    first.join();

}

bool serial_controller::addCallback(const callback_data_t &callback, const string data)
{
    if (hashmap.find(data) != hashmap.end())
    {
        return false;
    } else
    {
        hashmap[data] = callback;
        return true;
    }
}

bool serial_controller::enableDownload()
{
    mWriteMutex.lock();
    // Send SLD.
    ROS_INFO("Commanding driver to enter download mode.");
    // Set fals HLD mode
    isHLD = false;
    // Send enable write mode
    mSerial.write("%SLD 321654987" + eol);
    // Set lock variable and wait a data to return
    std::unique_lock<std::mutex> lck(mReaderMutex);
    // TODO change timeout
    cv.wait_for(lck, std::chrono::seconds(1));
    // Look for special ack from SLD.
    // ROS_INFO_STREAM("HLD=" << isHLD);
    // Unlock mutex
    mWriteMutex.unlock();
    // If not received return false
    if(!isHLD)
        return false;
}

bool serial_controller::downloadScript()
{
    if(enableDownload())
    {
        ROS_DEBUG("Writing script...");
        // Launch write script
        int line_num = 0;
        /*
        while(script_lines[line_num]) {
            // Build string
            std::string line(script_lines[line_num]);
            ROS_DEBUG_STREAM("write[" << line_num << "]" << line);
            bool cmd = command(line, "", "");
            // ROS_INFO_STREAM("cmd=" << (cmd ? "true" : "false"));
            // Check data and received a "+"
            if(!cmd)
                return false;
            // Go to second line
            line_num++;
        }
        */
        ROS_DEBUG("...complete!");
        return true;
    }
    return false;
}

bool serial_controller::command(string msg, string params, string type)
{
    // Lock the write mutex
    mWriteMutex.lock();
    // Build the string
    string msg2;
    if(params.compare("") == 0) {
        msg2 = type + msg + eol;
    } else {
        msg2 = type + msg + " " + params + eol;
    }
    unsigned int counter = 0;
    sub_data_cmd = false;
    while (counter < mMaxRetries)
    {
        mSerial.write(msg2.c_str());
        mCommandData = false;
        // Set lock variable and wait a data to return
        std::unique_lock<std::mutex> lck(mReaderMutex);
        cv.wait_for(lck, std::chrono::milliseconds(mResponseTimeoutMs), [this]{ return mCommandData; });
        if(mCommandData)
        {
            ROS_DEBUG_STREAM("N:" << (counter+1) << " CMD:" << msg << " DATA:" << sub_data_cmd);
            break;
        } else
        {
            // Increase counter
            counter++;
        }
    }
    // Unlock mutex
    mWriteMutex.unlock();
    return sub_data_cmd;
}

bool serial_controller::query(string msg, string params, string type) {
    mWriteMutex.lock();
    mMessage = msg;
    string msg2;
    if(params.compare("") == 0) {
        msg2 = type + msg + eol;
    } else {
        msg2 = type + msg + " " + params + eol;
    }

    unsigned int counter = 0;
    mQueryData = false;
    while (counter < mMaxRetries)
    {
        ROS_DEBUG_STREAM("N:" << (counter+1) << " TX: " << msg);
        mSerial.write(msg2.c_str());
        mQueryData = false;
        // Set lock variable and wait a data to return
        std::unique_lock<std::mutex> lck(mReaderMutex);
        cv.wait_for(lck, std::chrono::milliseconds(mResponseTimeoutMs), [this]{ return mQueryData; });
        if(mQueryData)
        {
            ROS_DEBUG_STREAM("N:" << (counter+1) << " CMD:" << msg << " DATA:" << sub_data);
            break;
        } else
        {
            // Increase counter
            counter++;
        }
    }
    // Clear last query request
    mMessage = "";
    // Unlock mutex
    mWriteMutex.unlock();
    return mQueryData;
}

void serial_controller::async_reader()
{
    while (!mStopping) {
        // Read how many byte waiting to read
        ROS_DEBUG_STREAM_NAMED("serial", "Bytes waiting: " << mSerial.available());
        // Read line
        std::string msg = mSerial.readline(max_line_length, eol);
        // Decode message
        if (!msg.empty())
        {
          boost::algorithm::trim_if(msg, boost::algorithm::is_any_of("\r\n"));
          if (msg.empty())
          {
              continue;
          }
          ROS_DEBUG_STREAM_NAMED("serial", "RX: " << msg);
          if (msg == "+" || msg == "-" || msg == "?+" || msg == "?-")
          {
              // Decode if command return true
              if(msg.find('+') != std::string::npos) sub_data_cmd = true;
              else sub_data_cmd = false;
              // Unlock command
              mCommandData = true;
              cv.notify_all();
          }
          else if(msg.find('=') != std::string::npos)
          {
              // Get command
              const size_t separator = msg.find('=');
              string sub_cmd = msg.substr(0, separator);
              if (!sub_cmd.empty() && sub_cmd[0] == '?')
              {
                  sub_cmd = sub_cmd.substr(1);
              }
              // Get data
              sub_data = msg.substr(separator + 1);
              // ROS_INFO_STREAM("CMD=" << sub_cmd << " DATA=" << sub_data);
              // Check first of all a message sent require a data to return
              if(mMessage.compare("") != 0) {
                  if(mMessage.compare(sub_cmd) == 0) {
                      mQueryData = true;
                      cv.notify_all();
                      // Skip other request
                      continue;
                  }
              }
              // Find in all callback a data to send
              if (hashmap.find(sub_cmd) != hashmap.end())
              {
                  // Get callback from hashmap
                  callback_data_t callback = hashmap[sub_cmd];
                  // Launch callback with return query
                  callback(sub_data);
              }
          }
          else if(msg == "HLD")
          {
              isHLD = true;
              // Unlock query request
              cv.notify_one();
          }
          else
          {
              ROS_DEBUG_STREAM_THROTTLE(1.0, "Unhandled serial message: " << msg);
          }
        }
    }
    ROS_INFO("Async serial reader closed");
}

}
