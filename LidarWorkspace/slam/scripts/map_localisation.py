#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import random
import yaml
import tf.transformations as tft
import matplotlib.pyplot as plt
import matplotlib.animation as animation
class MapLocalisation:
    def __init__(self, scan_topic, map_path, odom_topic, lidar_resolution, 
                 search_range, num_ransac_iterations, ransac_d_threshold, min_ransac_inliers, max_n_lines = 2):
        
        # Robot state
        self.pose = Pose()

        # Lidar parameters
        self.angle_increments = np.radians(360.0) / lidar_resolution
        self.lidar_resolution = lidar_resolution # Number of total readings that will be sent by lidar
        self.lidar_offset = int(np.radians(90) // self.angle_increments) # Lidar offset is -90 for real lidar and 90 for simulated in gazebo.
        self.search_range = search_range
        # Load map
        self.landmarks = self.load_map(map_path)

        # RANSAC Line detection parameters
        self.num_ransac_iterations = num_ransac_iterations
        self.ransac_d_threshold = ransac_d_threshold
        self.min_ransac_inliers = min_ransac_inliers
        self.max_n_lines = max_n_lines


        # Subscribers: Lidar Scan and Odometry topics
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)

        self.fig, self.ax = plt.subplots()
        self.scan_plot, = self.ax.plot([], [], 'bo', markersize=2)
        self.line_plots = []
        self.landmark_plots = []
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.show()

    def load_map(self, map_path):
        """
            Load a yaml file containing the landmarks' x and y positions.
            Return a list of landmark tuples (x, y)  
        """
        with open(map_path, 'r') as file:
            landmarks = yaml.load(file, Loader=yaml.FullLoader)
        names = landmarks['names']
        landmark_positions = []
        for name in names:
            if name in landmarks:
                x = landmarks[name]['x']
                y = landmarks[name]['y']
                landmark_positions.append((x, y))
        
        return landmark_positions

    def odometry_callback(self, msg):
        msg.pose.pose.position.x = msg.pose.pose.position.x
        msg.pose.pose.position.y = msg.pose.pose.position.y
        self.pose = msg.pose.pose

    def get_robot_yaw(self):
        robot_yaw = tft.euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])[2]
        return np.mod(robot_yaw + 2*np.pi, 2*np.pi)
        
    def get_landmarks_angular_positions(self): 
        """
            Returns the angular positions of the landmarks in the robots lidar reading frame
        """
        angular_positions = []
        
        # Robot orientation in inertial frame
        robot_yaw = self.get_robot_yaw()
        for landmark in self.landmarks:
            e_x = landmark[0] - self.pose.position.x
            e_y = landmark[1] - self.pose.position.y
            
            # Yaw orientation to the landmark from the robots current position
            landmark_orientation_inertial = np.mod(np.arctan2(e_y, e_x) + 2*np.pi, 2*np.pi)

            angle_difference = np.mod(landmark_orientation_inertial - robot_yaw + 2*np.pi, 2*np.pi)

            # Offset to account for the lidar offset
            angle_difference = np.mod(angle_difference + np.radians(90), 2*np.pi)
            angular_positions.append(angle_difference)


        return angular_positions
    
    def get_landmark_expected_scan_ranges_indexes(self): 
        """
            Return a list containing the scan indexes (start, end) to find each landmark in self.landmarks in the lidar scan.
            WARNING: start index could be greater than end index if the start angle is greater than the end angle, i.e. [3/2pi, 0]
        """

        angular_positions = self.get_landmarks_angular_positions() # In range [0 2pi]
        # print(angular_positions)
        range_slices = []
        for angle in angular_positions:
            range_start_angle = np.mod(angle - self.search_range / 2 + 2*np.pi, 2*np.pi)
            range_end_angle = np.mod(angle + self.search_range / 2 + 2*np.pi, 2*np.pi)
            print(range_start_angle * 180 / np.pi, range_end_angle * 180 / np.pi, self.lidar_offset)
            slice_start = self.lidar_offset + int(range_start_angle // self.angle_increments)
            slice_end = self.lidar_offset + int(range_end_angle // self.angle_increments)
            print(slice_start, slice_end)
            # Sanitize indexes
            slice_start = int(np.mod(slice_start, self.lidar_resolution))
            slice_end = int(np.mod(slice_end, self.lidar_resolution))
            range_slices.append((slice_start, slice_end))

        return range_slices
    
    def get_scan_data_from_slices(self, angle_start, angle_end, scan):
        if angle_start < angle_end:
            return scan[angle_start:angle_end + 1]
        return np.concatenate((scan[angle_start:], scan[0:angle_end + 1]))
    
    def ransac_line_fitting(self, points, num_iterations, distance_threshold, min_inliers, max_n_lines):
        """
            RANSAC line detection, return at most max_n_lines, sorted descending.
        """
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
        
        # Sort detected_lines and line_inliers by the length of inliers
        sorted_indices = np.argsort([len(inliers) for inliers in line_inliers])[::-1]
        detected_lines = [detected_lines[i] for i in sorted_indices]
        line_inliers = [line_inliers[i] for i in sorted_indices]
        
        return detected_lines[:max_n_lines], line_inliers[:max_n_lines]

    def get_lines_intersection(self, line1, line2):
        A = np.array([[line1[0], -1], [line2[0], -1]])
        b = np.array([-line1[1], -line2[1]])
        if np.linalg.det(A) == 0:
            return None 

        return np.linalg.solve(A, b)
    
    def scan_callback(self, data):
        scan = np.array(data.ranges)
        angles = data.angle_min + np.arange(len(scan)) * self.angle_increments
        landmark_n = np.arange(len(scan))
        
        # To plot
        self.points = np.empty((0, 2))
        self.lines = np.empty((0, 2))
        self.located_landmarks = np.empty((0, 2))

        for landmark, range, i in zip(self.landmarks, self.get_landmark_expected_scan_ranges_indexes(), landmark_n):
            print(f'Searching for landmark {i}')
            landmark_scan = self.get_scan_data_from_slices(range[0], range[1], scan)
            landmark_angles = self.get_scan_data_from_slices(range[0], range[1], angles)

            points = np.array([(r * np.cos(a), r * np.sin(a)) for r, a in zip(landmark_scan, landmark_angles)])

            # To plot
            self.points = np.concatenate((self.points, points))

            lines, inliers = self.ransac_line_fitting(points, 
                                                      num_iterations = self.num_ransac_iterations, 
                                                      distance_threshold = self.ransac_d_threshold, 
                                                      min_inliers = self.min_ransac_inliers, 
                                                      max_n_lines = self.max_n_lines)
            # To plot
            if len(lines) < 2:
                print(f'Failed to detect 2 lines for {landmark}')
                continue

            self.lines = np.concatenate((self.lines, lines))
            print(f'Detected {len(lines)} lines for {landmark}')
            print(lines)
            candidate_landmark = self.get_lines_intersection(lines[0], lines[1])
            print(f'Candidate landmark_point: {np.round(candidate_landmark,2)}')

            distance_from_expected_landmark_position = np.linalg.norm(np.array(landmark) - candidate_landmark)
            print(f'Distance from expected landmark position: {np.round(distance_from_expected_landmark_position)}')
            print()

            if abs(distance_from_expected_landmark_position) < 0.1:
                print(f'Landmark {i} detected at position {candidate_landmark}')
                self.located_landmarks = np.concatenate((self.located_landmarks, candidate_landmark.reshape(1, 2)))
            else:
                rospy.logwarn(f'Landmark {i} not detected')

        print()
        print()

    def update_plot(self, _):
        if hasattr(self, 'points'):
            self.scan_plot.set_data(self.points[:, 0], self.points[:, 1])
        
        while len(self.line_plots) > 0:
            ln = self.line_plots.pop()
            ln.remove()
        
        while len(self.landmark_plots) > 0:
            landmarks = self.landmark_plots.pop()
            for lm in landmarks:
                lm.remove()

        if hasattr(self, 'lines'):
            x = np.linspace(-4, 4, 100)
            for line in self.lines:
                y = line[0] * x + line[1]
                ln, = self.ax.plot(x, y, 'r-', linewidth=2)
                self.line_plots.append(ln)
            
            for lm in self.located_landmarks:
                landmark = self.ax.plot(lm[0], lm[1], 'go')
                self.landmark_plots.append(landmark)

                    # if self.have_points_near_intersection(self.inliers[i], intersection) and self.have_points_near_intersection(self.inliers[j], intersection):
                    #     print(self.compute_angle_between_lines(self.lines[i], self.lines[j]))
                    #     if np.abs(self.compute_angle_between_lines(self.lines[i], self.lines[j])) > 0.15:
                    #        self.intersections.append(intersection)
    
    # def have_points_near_intersection(self, points, intersection, threshold = 0.25):
    #     return np.any(np.linalg.norm(points - intersection, axis=1) < threshold)
    
    # def compute_angle_between_lines(self, line1, line2):
    #     return np.arctan((line2[0] - line1[0]) / (1 + line1[0] * line2[0]))
    

    # def update_plot(self, frame):
    #     if hasattr(self, 'points'):
    #         self.scan_plot.set_data(self.points[:, 0], self.points[:, 1])
        
    #     while len(self.line_plots) > 0:
    #         ln = self.line_plots.pop()
    #         ln.remove()
        
    #     while len(self.corner_plots) > 0:
    #         corners = self.corner_plots.pop()
    #         for corner in corners:
    #             corner.remove()

    #     if hasattr(self, 'lines'):
    #         x = np.linspace(-10, 10, 100)
    #         for line in self.lines:
    #             y = line[0] * x + line[1]
    #             ln, = self.ax.plot(x, y, 'r-', linewidth=2)
    #             self.line_plots.append(ln)
            
    #         for corner in self.intersections:
    #             corners = self.ax.plot(corner[0], corner[1], 'go')
    #             self.corner_plots.append(corners)

if __name__ == '__main__':
    rospy.init_node('map_localisation')

    # Set parameters and create map localisation object

    scan_topic = 'puzzlebot/scan'
    map_path = '/home/edgar/catkin_ws/src/T3003B_IntelligentRobotics/LidarWorkspace/slam/maps/gazebo_arena_landmarks.yaml'
    odom_topic = '/puzzlebot/odom'
    lidar_resolution = 1147
    num_ransac_iterations = 100
    ransac_d_threshold = 0.02
    min_ransac_inliers = 50
    search_range = np.radians(45)
    map_localisator = MapLocalisation(scan_topic, map_path, odom_topic, lidar_resolution, search_range, num_ransac_iterations, ransac_d_threshold, min_ransac_inliers)

    rospy.spin()
