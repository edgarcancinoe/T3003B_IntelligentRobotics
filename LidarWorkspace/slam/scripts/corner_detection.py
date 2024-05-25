#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

class RANSACCornerDetector:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/puzzlebot/scan', LaserScan, self.scan_callback)
        
        self.fig, self.ax = plt.subplots()
        self.scan_plot, = self.ax.plot([], [], 'bo', markersize=2)
        self.line_plots = []
        self.corner_plots = []
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.show()

    def ransac_line_fitting(self, points, num_iterations = 25, distance_threshold = 0.1, min_inliers = 180):

        detected_lines = []
        line_inliers = []
        for i in range(num_iterations):
            if len(points) < 2:
                break
            sample_indices = random.sample(range(len(points)), 2)
            sample = points[sample_indices]

            line = np.polyfit(sample[:, 0], sample[:, 1], 1)
            distances = np.abs(line[0] * points[:, 0] - points[:, 1] + line[1]) / np.sqrt(line[0]**2 + 1)
            inliers = points[distances < distance_threshold]

            if len(inliers) < min_inliers:
                continue
            
            # We have detected a candidate line            
            detected_lines.append(line)
            line_inliers.append(inliers.copy())

            # Subsample the inliers
            points = points[~np.isin(points, inliers).all(axis=1)]
            
        return detected_lines, line_inliers
    
    def get_line_intersections(self, line1, line2):
        A = np.array([[line1[0], -1], [line2[0], -1]])
        b = np.array([-line1[1], -line2[1]])
        return np.linalg.solve(A, b)
    
    
    def have_points_near_intersection(self, points, intersection, threshold = 0.25):
        return np.any(np.linalg.norm(points - intersection, axis=1) < threshold)
    
    def compute_angle_between_lines(self, line1, line2):
        return np.arctan((line2[0] - line1[0]) / (1 + line1[0] * line2[0]))
    
    def scan_callback(self, scan):
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = np.array(scan.ranges)
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        self.points = np.array([[r * np.cos(a), r * np.sin(a)] for r, a in zip(ranges, angles)])
        self.lines, self.inliers = self.ransac_line_fitting(self.points)
        
        if len(self.lines) > 1:
            # Get the intersection of all lines with each other
            self.intersections = []
            for i in range(len(self.lines)):
                for j in range(i + 1, len(self.lines)):
                    intersection = self.get_line_intersections(self.lines[i], self.lines[j])
                    if self.have_points_near_intersection(self.inliers[i], intersection) and self.have_points_near_intersection(self.inliers[j], intersection):
                        print(self.compute_angle_between_lines(self.lines[i], self.lines[j]))
                        if np.abs(self.compute_angle_between_lines(self.lines[i], self.lines[j])) > 0.15:
                           self.intersections.append(intersection)

        print(f"Detected {len(self.lines)} lines")
        print(f"Detected {len(self.intersections)} corners")
        print("CORNER LOCATIONS:")
        print(np.round(self.intersections, 2))
        print()

    def update_plot(self, frame):
        if hasattr(self, 'points'):
            self.scan_plot.set_data(self.points[:, 0], self.points[:, 1])
        
        while len(self.line_plots) > 0:
            ln = self.line_plots.pop()
            ln.remove()
        
        while len(self.corner_plots) > 0:
            corners = self.corner_plots.pop()
            for corner in corners:
                corner.remove()

        if hasattr(self, 'lines'):
            x = np.linspace(-10, 10, 100)
            for line in self.lines:
                y = line[0] * x + line[1]
                ln, = self.ax.plot(x, y, 'r-', linewidth=2)
                self.line_plots.append(ln)
            
            for corner in self.intersections:
                corners = self.ax.plot(corner[0], corner[1], 'go')
                self.corner_plots.append(corners)

if __name__ == '__main__':
    rospy.init_node('ransac_corner_detector')
    rcd = RANSACCornerDetector()
    rospy.spin()
