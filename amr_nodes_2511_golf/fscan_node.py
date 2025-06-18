#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
import statistics as sts

class ScanNodeG01(Node):
    def __init__(self):
        super().__init__("fscan_node")

        # Subscribing and publishing topics
        self.scan_subs = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, "/f_scan", 10)

        self.msg = LaserScan()
        self.plot = True
        self.filetered = True

        # For animated plotting
        if self.plot:
            plt.ion()
            self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
            self.line, = self.ax.plot([], [])
            self.ax.set_ylim(0, 16)
            self.ax.set_yticks([0.3] + list(range(2, 16, 2)))
            self.ax.yaxis.set_label_coords(-0.22, 0.5)
            self.ax.grid(True)
            self.ax.set_title("Lidar_plot", va='bottom')
            self.fig.show()
            self.fig.canvas.draw()
            self.line, = self.ax.plot([], [], color='red')


    def scan_callback(self, data: LaserScan):
        self.msg = data
        ranges = self.msg.ranges

        # Handle NaN and infinite values
        for i in range(len(ranges)):
            if math.isnan(ranges[i]):
                ranges[i] = 0.1  # Replace NaN with small value (e.g., 0.1)
            elif ranges[i] == math.inf:
                ranges[i] = 16.0  # Replace infinite values with maximum range

        # Apply median filtering to smooth the values
        for i in range(len(ranges)):
                self.msg.ranges[i] = self.median_filter(ranges, i, window_size=4)
            
        # Publish the filtered message
        self.scan_pub.publish(self.msg)

        if self.plot:
            if self.filetered:
                r= self.msg.ranges
            else:
                r = data.ranges

            r=r[270:]+r[:270]  # Rearrange the ranges to start from 0 degrees
            
            theta = np.linspace(data.angle_min, data.angle_max, len(r))

            self.line.set_data(theta, r)
            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        

    def median_filter(self, ranges, index, window_size=10):
        """
        Applies a median filter to the ranges at the given index.
        
        Args:
            ranges (list): List of LiDAR range values.
            index (int): The current index of the range value to filter.
            window_size (int): The size of the window for the median filter (default is 5).
            
        Returns:
            float: The median value of the window.
        """
        n = len(ranges)
        half_window = window_size // 2

        # Get indices with wrap-around
        indices = [(index + i) % n for i in range(-half_window, half_window + 1)]

        # Filter out invalid values (e.g., 15.0 as max range)
        valid_rays = [ranges[i] for i in indices if ranges[i] != 16.0]
        
        # If all values are invalid, return the default maximum range
        if len(valid_rays) == 0:
            return 16.0

        # Return the median of the valid values
        return sts.median(valid_rays)

def main(args=None):
    rclpy.init(args=args)

    # Create node object
    node = ScanNodeG01()

    # Keep the node running
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
