#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import random
import yaml
import tf.transformations as tft
from visualization_msgs.msg import Marker
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation

class MapLocalisation:
    def __init__(self, scan_topic, map_path, odom_topic, lidar_resolution, lidar_offset, inertial_frame_name, landmark_distance_threshold, 
                 search_range, num_ransac_iterations, ransac_d_threshold, min_ransac_inliers, max_n_lines = 2, visualize = True):
        
        # Robot state
        self.pose = Pose()

        # Lidar parameters
        self.angle_increments = np.radians(360.0) / lidar_resolution
        self.lidar_resolution = lidar_resolution # Number of total readings that will be sent by lidar
        self.lidar_offset = lidar_offset # Lidar offset in radians
        self.lidar_idx_offset = int(lidar_offset // self.angle_increments) # Offset to adjust the lidar readings to the robot's orientation
        self.search_range = search_range
        
        self.lidar_frame_angles = np.arange(lidar_resolution) * self.angle_increments
        self.lidar_to_robot_offset = np.radians(90) + lidar_offset
        self.robot_frame_angles = np.mod(self.lidar_frame_angles + self.lidar_to_robot_offset, 2*np.pi)

        # RANSAC Line detection parameters
        self.num_ransac_iterations = num_ransac_iterations
        self.ransac_d_threshold = ransac_d_threshold
        self.min_ransac_inliers = min_ransac_inliers
        self.max_n_lines = max_n_lines

        # Landmark detection
        self.landmark_distance_threshold = landmark_distance_threshold
        
        # Rviz visualization
        self.visualize = visualize
        self.frame_id = inertial_frame_name
        self.landmark_publisher = rospy.Publisher('landmark_positions', Marker, queue_size=10)

        # Load map
        self.landmarks = self.load_map(map_path)

        # Subscribers: Lidar Scan and Odometry topics
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)

        # self.fig, self.ax = plt.subplots()
        # self.scan_plot, = self.ax.plot([], [], 'bo', markersize=2)
        # self.line_plots = []
        # self.corner_plots = []
        # self.ax.set_xlim(-10, 10)
        # self.ax.set_ylim(-10, 10)
        
        # self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50)
        # plt.show()

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
        
        # Publish to rviz

        if self.visualize:
            rospy.sleep(4) # Wait for rviz to start
            for i, landmark in enumerate(landmark_positions):
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = rospy.Time.now()
                marker.ns = f'known landmark {i}'
                marker.id = i + len(landmark_positions)
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.pose.position.x = landmark[0]
                marker.pose.position.y = landmark[1]
                marker.pose.position.z = 0.05
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                marker.lifetime = rospy.Duration(0)
                self.landmark_publisher.publish(marker)

        return landmark_positions

    def odometry_callback(self, msg):
        self.pose = msg.pose.pose

    def get_robot_yaw(self):
        robot_yaw = tft.euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])[2]
        return np.mod(robot_yaw + 2*np.pi, 2*np.pi)
        
    def get_landmarks_angular_positions(self, robot_position, robot_yaw): 
        """
            Returns the angular positions of the landmarks in the robots lidar frame (assuming yaw=0 is at the right side of the robot)
        """
        angular_positions = []
        
        # Robot orientation in inertial frame
        for landmark in self.landmarks:
            e_x = landmark[0] - robot_position[0]
            e_y = landmark[1] - robot_position[1]
            
            # Yaw orientation to the landmark from the robots current position
            orientation_to_landmark_inertial = np.mod(np.arctan2(e_y, e_x) + 2*np.pi, 2*np.pi)
            # print() 
            # print('Orientation to landmark inertial: ', orientation_to_landmark_inertial)
            angle_difference = np.mod(orientation_to_landmark_inertial - robot_yaw + 2*np.pi, 2*np.pi)
            # print('Angle difference: ', angle_difference)
            # Offset to account for the lidar offset
            angle_difference = np.mod(angle_difference + np.radians(90), 2*np.pi)
            angular_positions.append(angle_difference)
        # print('Robot yaw: ', robot_yaw, 'Angular positions: ', angular_positions)
        return angular_positions
    
    def get_landmark_expected_scan_ranges_indexes(self, robot_position, robot_yaw): 
        """
            Return a list containing the scan indexes (start, end) to find each landmark in self.landmarks in the lidar scan.
            WARNING: start index could be greater than end index if the start angle is greater than the end angle, i.e. [3/2pi, 0]
        """

        angular_positions = self.get_landmarks_angular_positions(robot_position, robot_yaw) # In range [0 2pi]

        range_slices = []
        for angle in angular_positions:
            range_start_angle = np.mod(angle - self.search_range / 2 + 2*np.pi, 2*np.pi)
            range_end_angle = np.mod(angle + self.search_range / 2 + 2*np.pi, 2*np.pi)
            # print(range_start_angle * 180 / np.pi, range_end_angle * 180 / np.pi)
            slice_start = self.lidar_idx_offset + int(range_start_angle // self.angle_increments)
            slice_end = self.lidar_idx_offset + int(range_end_angle // self.angle_increments)
            # print(slice_start, slice_end)
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
        for _ in range(num_iterations):
            if len(points) < 2 or len(detected_lines) >= max_n_lines:
                break
            sample_indices = random.sample(range(len(points)), 2)
            sample = points[sample_indices]

            try:
                line = np.polyfit(sample[:, 0], sample[:, 1], 1)
            except np.linalg.LinAlgError:
                break
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
    
    def apply_robot2inertial_transform(self, points, rotation_matrix, translation_vector):
        """
            Transform points from the robot frame to the inertial frame
        """
        return np.dot(points, rotation_matrix.T) + translation_vector

    def scan_callback(self, data):
        # Get scan data as np array
        scan = np.array(data.ranges)
        
        # Compute instantaneous robot yaw and transformation parameters
        robot_yaw = self.get_robot_yaw()
        rotation_matrix = np.array([[np.cos(robot_yaw), -np.sin(robot_yaw)], 
                                           [np.sin(robot_yaw), np.cos(robot_yaw)]])
        robot_position = np.array([self.pose.position.x, self.pose.position.y])

        angular_ranges = self.get_landmark_expected_scan_ranges_indexes(robot_position, robot_yaw)

        # To plot
        # self.lines = np.empty((0, 2))
        # self.points = np.empty((0, 2))
        self.located_landmarks = np.empty((0, 2))

        for i, (landmark, angular_range) in enumerate(zip(self.landmarks, angular_ranges)):
            # print(f'Searching for landmark {i}')
            landmark_scan = self.get_scan_data_from_slices(angular_range[0], angular_range[1], scan)
            landmark_angles = self.get_scan_data_from_slices(angular_range[0], angular_range[1], self.robot_frame_angles)

            points_robot_frame = np.array([(r * np.cos(a), r * np.sin(a)) for r, a in zip(landmark_scan, landmark_angles)])
            points = self.apply_robot2inertial_transform(points_robot_frame, rotation_matrix, robot_position)
            
            lines, inliers = self.ransac_line_fitting(points, 
                                                      num_iterations = self.num_ransac_iterations, 
                                                      distance_threshold = self.ransac_d_threshold, 
                                                      min_inliers = self.min_ransac_inliers, 
                                                      max_n_lines = self.max_n_lines)
            
            # if self.visualize:
            #     self.lines = np.concatenate((self.lines, lines))
            #     self.points = np.concatenate((self.points, points))

            if len(lines) < 2:
                # rospy.logwarn(f'Failed to detect 2 lines for {i}')
                continue

            candidate_landmark = self.get_lines_intersection(lines[0], lines[1])
            distance_from_expected_landmark_position = np.linalg.norm(np.array(landmark) - candidate_landmark)

            # print(f'Distance from expected landmark position: {np.round(distance_from_expected_landmark_position,3)}')
            # print(f'Landmark {i} detected at position {candidate_landmark}')
            # print(f'Landmark {i} expected at position {landmark}')
            if abs(distance_from_expected_landmark_position) < self.landmark_distance_threshold:

                print(f'Landmark {i} detected at position {candidate_landmark}')
                self.located_landmarks = np.concatenate((self.located_landmarks, candidate_landmark.reshape(1, 2)))
                pass
            # else:
                # rospy.logwarn(f'Landmark {i} not detected')
                # pass
        if self.visualize:
            self.send_visualization_markers()
        
    def send_visualization_markers(self):
        for i, landmark in enumerate(self.located_landmarks):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'landmarks'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.pose.position.x = landmark[0]
            marker.pose.position.y = landmark[1]
            marker.pose.position.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(1)

            self.landmark_publisher.publish(marker)

    # def update_plot(self, _):
    #     if hasattr(self, 'points'):
    #         self.scan_plot.set_data(self.points[:, 0], self.points[:, 1])
        
    #     while len(self.line_plots) > 0:
    #         ln = self.line_plots.pop()
    #         ln.remove()

    #     if hasattr(self, 'lines'):
    #         x = np.linspace(-4, 4, 100)
    #         for line in self.lines:
    #             y = line[0] * x + line[1]
    #             ln, = self.ax.plot(x, y, 'r-', linewidth=2)
    #             self.line_plots.append(ln)
            
                    # if self.have_points_near_intersection(self.inliers[i], intersection) and self.have_points_near_intersection(self.inliers[j], intersection):
                    #     print(self.compute_angle_between_lines(self.lines[i], self.lines[j]))
                    #     if np.abs(self.compute_angle_between_lines(self.lines[i], self.lines[j])) > 0.15:
                    #        self.intersections.append(intersection)
    
    # def have_points_near_intersection(self, points, intersection, threshold = 0.25):
    #     return np.any(np.linalg.norm(points - intersection, axis=1) < threshold)
    
    # def compute_angle_between_lines(self, line1, line2):
    #     return np.arctan((line2[0] - line1[0]) / (1 + line1[0] * line2[0]))
    
if __name__ == '__main__':
    rospy.init_node('map_localisation')

    # Set parameters and create map localisation object

    scan_topic = '/scan'
    map_path = '/home/puzzlebot/catkin_ws/src/T3003B_IntelligentRobotics/LidarWorkspace/slam/maps/real_arena_landmarks.yaml'
    odom_topic = '/puzzlebot/odom'
    lidar_resolution = 1147
    lidar_offset = -np.radians(90) # Lidar offset is -90 for real lidar and 90 for simulated in gazebo.
    num_ransac_iterations = 70
    ransac_d_threshold = 0.025
    min_ransac_inliers = 30
    search_range = np.radians(60)
    inertial_frame_name = 'odom'
    landmark_distance_threshold = .4
    map_localisator = MapLocalisation(scan_topic, map_path, odom_topic, lidar_resolution, lidar_offset, inertial_frame_name, landmark_distance_threshold,
                                      search_range, num_ransac_iterations, ransac_d_threshold, min_ransac_inliers)

    rospy.spin()
