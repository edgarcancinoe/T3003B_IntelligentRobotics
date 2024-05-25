#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Bool, Float32
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import tf.transformations as tft

class Navigator:
    def __init__(self, 
                 odom_topic, lidar_topic,
                 pose_controller_activate_topic, pose_controller_topic,
                 orientation_controller_activate_topic, orientation_controller_topic,
                 cmd_vel_topic, unlock_topic,
                 horizontal_tolerance, frontal_tolerance,
                 lidar_resolution,
                 linear_v, rotation_w, goal_tolerance, turn_direction = 'left', lidar_angle_min = -30, lidar_angle_max = 210):
        
        # Navigator parameters
        self.linear_v = linear_v
        self.rotation_w = rotation_w
        self.direction = 1 if turn_direction == 'left' else -1

        # State (OrientateToGoal, MovingTowardsGoal, FollowWall)
        self.state = None
        self.unlocked = True
        self.final_goal = Pose(position=Point(0,0,0), orientation=Quaternion(0,0,0,1))
        self.goal_reached = False

        # Lidar detection variables
        self.collision_ahead = False
        self.free_way = False
        self.front_free = False
        self.right_free = False
        self.left_free = False
        self.goal_tolerance = goal_tolerance
        self.lidar_angle_min = lidar_angle_min
        self.lidar_angle_max = lidar_angle_max
        
        # Dynamic safe space for ahead movement
        self.angle_step = 360 / lidar_resolution

        x_lim = horizontal_tolerance
        R = frontal_tolerance

        self.theta_1 = np.round(np.degrees(np.arccos(x_lim / R)))
        self.theta_2 = 180 - self.theta_1

        self.max_front_distances, self.max_left_distances, self.max_right_distances = self._get_front_left_right_scan_max_ranges(self.theta_1, self.theta_2, R, x_lim)
        
        self.front_slice, self.left_slice, self.right_slice = self._get_front_left_right_scan_slices()

        # Odometry state    
        self.pose = Pose()

        # Subscribers
        rospy.Subscriber(odom_topic, Odometry, self._odom_callback)
        rospy.Subscriber(lidar_topic, LaserScan, self._lidar_callback)
        rospy.Subscriber(unlock_topic, Bool, self._unlock_callback)

        # Publishers ------------ 

        # Pose controller activation and message
        self.pose_controller_activate_pub = rospy.Publisher(pose_controller_activate_topic, Bool, queue_size=10)
        self.pose_controller_pub = rospy.Publisher(pose_controller_topic, Pose, queue_size=10)
        # Orientation controller activation and message
        self.orientation_controller_activate_pub = rospy.Publisher(orientation_controller_activate_topic, Bool, queue_size=10)
        self.orientation_controller_pub = rospy.Publisher(orientation_controller_topic, Float32, queue_size=10)
        # Command velocity
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    def _get_front_left_right_scan_max_ranges(self, theta_1, theta_2, R, x_lim):

        # Safe space for ahead movement 
        n_points = int((theta_2 - theta_1) / self.angle_step)

        # Front: from theta_1 to theta_2 degrees, safe if d > R
        max_front_distances = np.array([R for _ in range(n_points)])

        # Sides: from angle_min to theta_1 and theta_2 to angle_max degrees, safe if d > x_lim / cos(theta)
        
        side_readings_length = int(np.ceil((theta_1 - self.lidar_angle_min) / self.angle_step - 1))
        max_right_distances = np.array([x_lim / np.cos(np.radians(n * self.angle_step)) for n in range(side_readings_length)])
        max_left_distances = np.flip(max_right_distances)

        return max_front_distances, max_left_distances, max_right_distances
    
    def _get_front_left_right_scan_slices(self, offset=-90):
        """
            Returns the slices for the front, left and right lidar readings.
            The slices are defined by the angles theta_1 and theta_2, and the offset  (degrees) is used to shift the slices if needed.
        """

        
        # Lidar readings start at robots back so we need to add an offset to the angles
        offset = int(offset // self.angle_step)

        angle_min_idx = offset + int(self.lidar_angle_min // self.angle_step)
        theta1_idx = offset + int(self.theta_1 // self.angle_step)
        theta2_idx = offset + int(self.theta_2 // self.angle_step)
        pi_idx = offset + int(self.lidar_angle_max // self.angle_step)
 
        front_slice = slice(theta1_idx, theta2_idx, 1)
        left_slice = slice(theta2_idx + 1, pi_idx, 1)
        right_slice = slice(angle_min_idx, theta1_idx - 1, 1)

        # Ensure left and right slices have the same length
        min_length = min(len(range(left_slice.start, left_slice.stop)), len(range(right_slice.start, right_slice.stop)))
        left_slice = slice(left_slice.start, left_slice.start + min_length, left_slice.step)
        right_slice = slice(right_slice.start, right_slice.start + min_length, right_slice.step)

        return front_slice, left_slice, right_slice
    
    def _collision_in(self, detections, distances, percentage=0.03):
        """
            Returns True if more than <percentage>% of the readings 
            in detections are less than the corresponding distances.
        """
        return np.sum(detections < distances)/len(detections) > percentage
    
    def _sanitize_detections(self, data, slice_):
        if slice_.start > 0 and slice_.stop > slice_.start: # Both are positive and stop is greater than start
            return data[slice_]
        elif slice_.start > 0 and slice_.stop < 0: # Start is positive and stop is negative
            return data[slice_.start:slice_.stop]
        elif slice_.start < 0 and slice_.stop > 0: # Start is negative and stop is positive
            return np.concatenate((data[slice_.start:], data[0:slice_.stop]))
        elif slice_.start < 0 and slice_.stop < 0: # Both are negative
            return data[slice_.start:slice_.stop]
        else:
            raise ValueError('Invalid slice')

    def _lidar_callback(self, msg):
        
        collision = False
        front_free = True
        left_free = True
        right_free = True

        front_detections = self._sanitize_detections(msg.ranges, self.front_slice)
        left_detections = self._sanitize_detections(msg.ranges, self.left_slice)
        right_detections = self._sanitize_detections(msg.ranges, self.right_slice)

        if self._collision_in(front_detections, self.max_front_distances):
            rospy.logwarn_once('Collision ahead (front)')
            collision = True
            front_free = False
        if self._collision_in(left_detections, self.max_left_distances):
            rospy.logwarn_once('Collision ahead (left)')
            collision = True
            left_free = False
        if self._collision_in(right_detections, self.max_right_distances):
            rospy.logwarn_once('Collision ahead (right)')
            collision = True
            right_free = False
        
        self.collision_ahead = collision
        self.front_free = front_free
        self.left_free = left_free
        self.right_free = right_free

    def _euclidean_distance(self, p1, p2):
        return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
    
    def _odom_callback(self, msg):
            self.pose = msg.pose.pose

            if self.state == 'FollowWall':
                if self._euclidean_distance(self.pose.position, self.final_goal.position) < self.goal_tolerance:
                    self.goal_reached = True

    def _unlock_callback(self, msg):
        if self.state == 'OrientateToGoal' or self.state == 'MovingTowardsGoal':
            if msg.data:
                rospy.logwarn('Unlocked: Goal reached')

        self.unlocked = msg.data

    def _activate_pose_controller(self):
        self.pose_controller_activate_pub.publish(Bool(True))

    def _deactivate_pose_controller(self):
        self.pose_controller_activate_pub.publish(Bool(False))
    
    def _activate_orientation_controller(self):
        self.orientation_controller_activate_pub.publish(Bool(True))
    
    def _deactivate_orientation_controller(self):
        self.orientation_controller_activate_pub.publish(Bool(False))
    
    def _set_goal_pose(self, goal_pose):
        self._activate_pose_controller()
        self.unlocked = False
        self.pose_controller_pub.publish(goal_pose)
    
    def _set_goal_orientation(self, goal_orientation):
        """
            Orientation is a float message, in the range [0 2*pi]
        """
        self._activate_orientation_controller()
        self.unlocked = False
        self.orientation_controller_pub.publish(Float32(goal_orientation))

    def _get_equation_of_line_to_goal(self, goal_pose: Pose):
        """
        Returns the equation of the line connecting the robot and the goal pose
        y = mx + b
        """
        m = (goal_pose.position.y - self.pose.position.y) / (goal_pose.position.x - self.pose.position.x)
        b = self.pose.position.y - m * self.pose.position.x
        return m, b
    
    def _set_goal(self, goal_pose: Pose):
        self.final_goal = goal_pose
        rospy.logwarn(f'Goal set to {goal_pose.position.x}, {goal_pose.position.y}')

    def _yaw_to_point(self, point):
        """
            Returns the yaw orientation to point towards a point from the robot's current position
        """
        yaw = np.arctan2(point.y - self.pose.position.y, point.x - self.pose.position.x)
        return np.mod(yaw + 2*np.pi, 2*np.pi)
    
    def _final_goal_orientation_yaw(self):
        """
            Returns the yaw orientation of the final goal
        """
        goal_yaw = tft.euler_from_quaternion([self.final_goal.orientation.x, self.final_goal.orientation.y, self.final_goal.orientation.z, self.final_goal.orientation.w])
        goal_yaw = np.mod(goal_yaw[2] + 2*np.pi, 2*np.pi)
        return goal_yaw
    
    def _orientate_towards_goal(self):
        """
            Orientate the robot towards the goal and wait until the orientation is achieved
        """
        if self.final_goal is not None:
            self.state = 'OrientateToGoal'
            self._activate_orientation_controller()
            self._set_goal_orientation(self._final_goal_orientation_yaw())
            while not self.unlocked and not rospy.is_shutdown():
                rospy.sleep(0.1)
            self._deactivate_orientation_controller()
            rospy.logwarn('Orientated towards goal')
            return True
        else:
            raise ValueError('No goal set')

    def _wait_until_goal_reached_or_collision_warning(self):
        rospy.logwarn(f'Waiting until process is finished ({self.state}) or collision is detected...')
        while not self.unlocked and not rospy.is_shutdown() and not self.collision_ahead:
            rospy.sleep(0.1)
        
        if self.unlocked:
            return True
        elif self.collision_ahead:
            return False
    
    def _rotate_until_front_is_free(self):
        rospy.logwarn('Turning left until front is free...')
        while not self.front_free and not rospy.is_shutdown():
            self._turn()
        if self.front_free:
            return True
        else:
            return False
    
    def _free_way_detected(self) -> bool:
        # Implement method on child classes
        raise NotImplementedError('Method not implemented')
    
    def _find_wall(self):
        self._send_cmd_vel(self.linear_v * 0.6, -self.direction * self.rotation_w)
        rospy.sleep(0.1)

    def _move_forward(self):
        self._send_cmd_vel(self.linear_v, 0.0)
        rospy.sleep(0.1)

    def _turn(self):
        self._send_cmd_vel(0.0, self.direction * self.rotation_w)
        rospy.sleep(0.1)

    def _follow_wall(self):
        """
            Follow the wall until the goal is reached or a free way is detected
            returns true if the goal is reached and false if a free way is detected
        """

        self._align_parallel_to_wall()

        while not rospy.is_shutdown():
            if self.goal_reached:
                return True
            if self._free_way_detected():
                return False
            else:
                # Check state and move accordingly
                if self.front_free and self.left_free and self.right_free:
                    self._find_wall()
                elif not self.front_free and self.left_free and self.right_free:
                    self._turn()
                elif self.front_free and self.left_free and not self.right_free:
                    self._move_forward()
                elif self.front_free and not self.left_free and self.right_free:
                    self._find_wall()
                elif not self.front_free and self.left_free and not self.right_free:
                    self._turn()
                elif not self.front_free and not self.left_free and self.right_free:
                    self._turn()
                elif not self.front_free and not self.left_free and not self.right_free:
                    self._turn()
                elif self.front_free and not self.left_free and not self.right_free:
                    self._move_forward()

    # Robot commands
    def _stop_robot(self):
        self._deactivate_pose_controller()
        self.cmd_vel_pub.publish(Twist(linear=Point(0,0,0), angular=Point(0,0,0)))

    def _send_cmd_vel(self, v, w):
        self.cmd_vel_pub.publish(Twist(linear=Point(v,0,0), angular=Point(0,0,w)))

    def _align_parallel_to_wall(self):
        self._rotate_until_front_is_free()
        rospy.sleep(0.25)
        self._stop_robot()

    def _move_towards_goal(self):
        # Send pose goal to pose controller
        if self.final_goal is not None:
            
            # Move towards goal and set the state to MovingTowardsGoal
        
            self.state = 'MovingTowardsGoal'
            self._activate_pose_controller()
            self._set_goal_pose(self.final_goal)

            reached_goal = False
            
            while not reached_goal:
                if self.state == 'MovingTowardsGoal':
                    # Will continue moving until the goal is reached or a collision is detected
                    reached_goal = self._wait_until_goal_reached_or_collision_warning()
                    
                    if reached_goal:
                        rospy.logwarn('Goal reached')
                        return True
                    
                    else: # Collision detected
                        
                        # Stop the robot
                        self._stop_robot()

                        rospy.logwarn('Collision detected. Switching to FollowWall state')
                        self.state = 'FollowWall'
                        continue
                    
                elif self.state == 'FollowWall':
                    # Follow wall until the goal is reached
                    rospy.logwarn('Following wall...')
                    reached_goal = self._follow_wall()

                    if reached_goal:
                        rospy.logwarn('Goal reached')
                        return True
                    
                    else: # Free way detected
                        
                        # Stop the robot
                        self._stop_robot()
                        rospy.logwarn('Free way detected. Switching to MovingTowardsGoal state')
                        rospy.sleep(1)
                        self._orientate_towards_goal()
                        self.state = 'MovingTowardsGoal'
                        self._activate_pose_controller()
                        self._set_goal_pose(self.final_goal)
                        continue
        else:
            raise ValueError('No goal set')
    
    def go_to_pose(self, pose):
        self._set_goal(pose)
        self._orientate_towards_goal()
        self._move_towards_goal()

    def go_to_point(self, point):
        orientation = tft.quaternion_from_euler(0,0,navigator._yaw_to_point(point))
        pose = Pose(position=point, 
                    orientation=Quaternion(*orientation))
        self._set_goal(pose)
        self._orientate_towards_goal()
        self._move_towards_goal()
    
class Bug2Nav(Navigator):
    def __init__(self, *args, min_d_hitpoint=0.04):
        super(Bug2Nav, self).__init__(*args)        
        self.min_d_hitpoint = min_d_hitpoint # Minimum distance to hitpoint to leave wall
        self.hitpoint = None
        self.d_hitpoint_to_goal = None

    def _set_goal(self, goal_pose: Pose):
        super()._set_goal(goal_pose)
        self.m, self.b = self._get_equation_of_line_to_goal(goal_pose)
    
    def _follow_wall(self):
        self.hitpoint = self.pose.position
        self.d_hitpoint_to_goal = self._euclidean_distance(self.hitpoint, self.final_goal.position)
        super()._follow_wall()

    def _on_goal_line(self):
        return np.abs(self.m * self.pose.position.x - self.pose.position.y + self.b) < self.goal_tolerance
    
    def _free_way_detected(self) -> bool:
        # Leave wall if the robot is on the goal line and the distance to the goal is less than the distance hitpoint to goal
        if self._on_goal_line():
            rospy.logwarn('Found line to goal')
            distance_to_goal = self._euclidean_distance(self.pose.position, self.final_goal.position)

            if abs(distance_to_goal - self.d_hitpoint_to_goal) > self.min_d_hitpoint:
                rospy.logwarn('Free way detected')
                return True
            else:
                rospy.logwarn('At previous hitpoint')
        return False

class Bug0Nav(Navigator):
    def __init__(self, *args):
        super(Bug0Nav, self).__init__(*args)
        self.free_way_to_goal = None
        
    def _lidar_callback(self, msg):
        super()._lidar_callback(msg)
        self.free_way_to_goal = self._free_way_to_goal(msg)

    def _free_way_to_goal(self, scan):
        # Angle between robot and goal
        goal_yaw = self._yaw_to_point(self.final_goal.position)
        
        # Robot orientation in [0 2pi]
        robot_yaw = tft.euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])[2]
        robot_yaw = robot_yaw + np.pi/2 # Correct orientation so that 0 is at the right side of the robot to follow adopted convention
        robot_yaw = np.mod(robot_yaw + 2*np.pi, 2*np.pi)

        angle_difference = goal_yaw - robot_yaw

        if angle_difference > np.pi:
            return False
        
        target_front_slice, target_left_slice, target_right_slice = self._get_front_left_right_scan_slices(offset=np.degrees(angle_difference))

        # Check if there is a free way to the goal
        front_detections = self._sanitize_detections(scan.ranges, target_front_slice)
        left_detections = self._sanitize_detections(scan.ranges, target_left_slice)
        right_detections = self._sanitize_detections(scan.ranges, target_right_slice)

        frontal_collision = self._collision_in(front_detections, self.max_front_distances)
        left_collision = self._collision_in(left_detections, self.max_left_distances)
        right_collision = self._collision_in(right_detections, self.max_right_distances)

        return not frontal_collision and not left_collision and not right_collision
    
    def _free_way_detected(self) -> bool:
        return self.free_way_to_goal
        
        
if __name__ == '__main__':
    rospy.init_node('bug_navigator', anonymous=True)

    odometry_topic = rospy.get_param('/sim_odom_topic')
    lidar_topic = rospy.get_param('/lidar_topic')
    pose_controller_topic = rospy.get_param('/pose_controller_topic')
    pose_controller_activate_topic = rospy.get_param('/pose_control_activate_topic')
    orientation_controller_topic = rospy.get_param('/orientation_controller_topic')
    orientation_controller_activate_topic = rospy.get_param('/orientation_control_activate_topic')
    unlock_topic = rospy.get_param('/unlock_topic')
    cmd_vel_topic = rospy.get_param('/commands_topic')
    
    goal_tolerance = rospy.get_param('/pose_controller/r_tolerance') * 1.35

    horizontal_tolerance = rospy.get_param('~horizontal_tolerance')
    frontal_tolerance = rospy.get_param('~frontal_tolerance')
    lidar_resolution = rospy.get_param('~lidar_resolution')
    v = rospy.get_param('~v')
    w = rospy.get_param('~w')

    min_d_hitpoint = rospy.get_param('~bug2/min_d_hitpoint')

    navigator = Bug0Nav(odometry_topic, lidar_topic, 
                     pose_controller_activate_topic, pose_controller_topic,
                     orientation_controller_activate_topic, orientation_controller_topic, 
                     cmd_vel_topic, unlock_topic,
                     horizontal_tolerance, frontal_tolerance,
                     lidar_resolution, v, w, goal_tolerance, 'right')
    
    #navigator = Bug2Nav(odometry_topic, lidar_topic, 
    #                pose_controller_activate_topic, pose_controller_topic,
    #                orientation_controller_activate_topic, orientation_controller_topic, 
    #                cmd_vel_topic, unlock_topic,
    #                horizontal_tolerance, frontal_tolerance,
    #                lidar_resolution, v, w, goal_tolerance, 'left', min_d_hitpoint=min_d_hitpoint)
    
    rospy.sleep(3)

    point = Point(5.25, -2.20, 0.0)
    navigator.go_to_point(point)

    # Spin the node
    rospy.spin()


