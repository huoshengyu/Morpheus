#! /usr/bin/env python3

import rospy
import sensor_msgs.msg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np

import sys

class Analysis():

    def __init__(self, filename = None):
        self.title = None
        self.column_labels = None
        self.data = None
        self.column_label_dict = None
        if filename is not None:
            self.read(filename)
        

    def read(self, filename):
        self.filename = filename
        self.column_labels = np.genfromtxt(filename, delimiter=",", dtype='unicode', max_rows=1, autostrip=True)
        print(self.column_labels)
        self.column_label_dict = {}
        for i in range(len(self.column_labels)):
            self.column_label_dict[self.column_labels[i]] = i
        self.data = np.array(np.genfromtxt(filename, delimiter=",", dtype=None, skip_header=1, autostrip=True, unpack=True))
        print(np.shape(self.data))

    def plot(self):
        if self.data is None:
            print("No data to plot! Run read(filename) first.")
            return
        
        # Plot all data for sanity checking
        plt.figure("All columns")
        for i in range(1, len(self.column_labels)):
            plt.plot(self.data[0], self.data[i])
        
        # Plot distance to nearest collision
        plt.figure("Distance to collision")
        plt.plot(self.data[0], self.data[self.column_label_dict["nearest_collision_depth"]])

        def update_lines(step, line_data, line_plots):
            for line_plot, line_datum in zip(np.array(line_plots), np.array(line_data)):
                line_plot.set_data(line_datum[:step,:2].T)
                line_plot.set_3d_properties(line_datum[:step,2])
            return line_plots

        # Set info for plotting position over time
        num_steps = len(self.data[:,0])
        links = ["tcp_link"]
        position_suffixes = ["_position_x", "_position_y", "_position_z"]
        link_positions = [[link + suffix for suffix in position_suffixes] for link in links]
        line_data = [[self.data[self.column_label_dict[position]] for position in link] for link in link_positions]

        # Attaching 3D axis to the figure
        fig = plt.figure("tcp_link position")
        ax = fig.add_subplot(projection="3d")
        
        # Create lines initially without data
        line_plots = [ax.plot([], [], [])[0] for _ in links]

        # Setting the axes properties
        ax.set(xlim3d=(-1, 1), xlabel='X')
        ax.set(ylim3d=(-1, 1), ylabel='Y')
        ax.set(zlim3d=(-1, 1), zlabel='Z')

        # Creating the Animation object
        ani = animation.FuncAnimation(
        fig, update_lines, num_steps, fargs=(line_data, line_plots), interval=100)

        plt.show()

        

if __name__=="__main__":

    filename = "/root/catkin_ws/src/morpheus/morpheus_data/data/UR5e_simkeyhole_0_2024-03-12_16h49m22s.csv"
    if len(sys.argv) > 1:
        filename = sys.argv[1]

    analysis = Analysis(filename)
    analysis.plot()
