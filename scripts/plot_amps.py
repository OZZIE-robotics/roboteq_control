#!/usr/bin/env python3
import os
import rospy
from roboteq_control.msg import ControllerAmps
import matplotlib.pyplot as plt
import threading

class ControllerAmpsPlotter:
    def __init__(self):
        self.time_data = []
        self.controller1_amps = []
        self.controller2_amps = []
        self.lock = threading.Lock()

        rospy.init_node('amps_plotter')
        rospy.Subscriber('/complete_roboteq/controller_amps', ControllerAmps, self.callback)

    def callback(self, data):
        with self.lock:
            self.time_data.append(rospy.Time.now().to_sec())
            # Store first channel amps as example; extend as needed for all channels
            if len(data.controller1_amps) > 0:
                self.controller1_amps.append(data.controller1_amps[0])
            else:
                self.controller1_amps.append(0.0)
            if len(data.controller2_amps) > 0:
                self.controller2_amps.append(data.controller2_amps[0])
            else:
                self.controller2_amps.append(0.0)

    def plot_data(self):
        with self.lock:
            if len(self.time_data) < 2:
                print("Insufficient data to plot.")
                return
            plt.plot(self.time_data, self.controller1_amps, label='Controller 1 Amps')
            plt.plot(self.time_data, self.controller2_amps, label='Controller 2 Amps')
            plt.xlabel('Time (seconds)')
            plt.ylabel('Amps')
            plt.title('Roboteq SBLG2360T Controller Amps')
            plt.legend()
            plt.savefig('/home/rover/amps_plot.png')
            print("Saving plot to:", os.getcwd())

def shutdown_hook():
    print("Shutdown detected, plotting data...")
    plotter.plot_data()

if __name__ == '__main__':
    rospy.init_node('amps_plotter')
    plotter = ControllerAmpsPlotter()
    rospy.on_shutdown(shutdown_hook)
    rospy.spin()
