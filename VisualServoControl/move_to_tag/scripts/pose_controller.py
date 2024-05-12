#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Vector3, Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from puzzlebot_util.util import *
import math
import tf.transformations as tf

class Puzzlebot_controller():
    def __init__(self, starting_pose, starting_orientation,
                 k1, k2, beta, lambda_, v_max, 
                 r_tolerance, commands_topic, odom_topic, goals_topic, 
                 send_done_topic, pose_control_activate_topic, control_rate):

        # Control parameters
        self.k1 = k1
        self.k2 = k2
        self.beta = beta
        self.lambda_ = lambda_
        self.v_max = v_max

        # Reference (goal)
        self.s_d : Pose = None

        # Initial state (inertial frame)
        self.s : Pose = Pose(position=starting_pose, 
                             orientation=tf.quaternion_from_euler(0.0, 0.0, starting_orientation))

        # Set tolerances 
        self.r_tolerance : float = r_tolerance
        
        # Publishers
        self.cmd_vel_publisher = rospy.Publisher(commands_topic, Twist, queue_size=10)
        self.reached_goal_publisher = rospy.Publisher(send_done_topic, Bool, queue_size=10)
        self.error_publisher = rospy.Publisher('/pose_controller_error', Float32, queue_size=10)
        # Suscriber
        rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)
        rospy.Subscriber(goals_topic, Pose, self.set_goal)
        rospy.Subscriber(pose_control_activate_topic, Bool, self._activate_controller)

        # Goals        
        self.reached_goal = False

        self.active = False

        rospy.Timer(rospy.Duration(1.0/control_rate), self.compute_output)

    def _activate_controller(self, msg):
        if msg.data:
            rospy.logwarn(f'Pose controller activated')
        else:
            rospy.logwarn(f'Pose controller deactivated')
        self.active = msg.data

    def orientation_as_euler(self, quat):
        euler = tf.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        # roll = euler[0]
        # pitch = euler[1]
        yaw = euler[2]
        # return roll, pitch, yaw
        return yaw
    
    def odometry_callback(self, msg):
        self.s = msg.pose.pose
        self.s.orientation = Quaternion(x=0.0, y=0.0, z=self.orientation_as_euler(self.s.orientation), w=1.0)

    def reset(self):
        self.ei_l = 0.0
        self.ei_w = 0.0
        self.e_l_prev = 0.0
        self.e_w_prev = 0.0
        self.s_d = None
        self.reached_goal = False

    def set_goal(self, goal: Point):
        self.ei_l = 0.0
        self.ei_w = 0.0
        self.e_l_prev = 0.0
        self.e_w_prev = 0.0
        self.s_d = goal
        rospy.logwarn(f'\nReaching for goal:\n {self.s_d}')

    def goal_set(self):
        return self.s_d != None
    
    def _get_descriptive_state_variables(self):
        # Distance error
        e_x = self.s_d.position.x - self.s.position.x
        e_y = self.s_d.position.y - self.s.position.y
        r = np.sqrt((e_x * e_x) + (e_y * e_y))

        # Angular variables
        target_z_orientation = self.orientation_as_euler(self.s_d.orientation)

        # Line of sight angle
        angle = np.arctan2(e_y, e_x)
        # Angle pose to line of sight
        theta = -(angle - target_z_orientation)
        delta = -(angle - self.s.orientation.z)
        virtual_control_delta = np.arctan(-self.k1 * theta)

        # Curvature
        K = - 1 / r * (self.k2 * (delta - virtual_control_delta) + (1 + self.k1 / (1 + (self.k1 * theta)**2)) * np.sin(theta))

        return r, theta, delta, virtual_control_delta, K

    def _dynamic_velocity(self, K):
        """ Linear velocity function:

            Decreases as the curvature increases. The robot will slow down as it approaches the goal as K is proportional to 1/r
            v = v_max / (1 + beta * |K|^lambda)
            
            K: Curvature
            v_max: Maximum linear velocity

        """
        return self.v_max / (1 + self.beta * np.abs(K) ** self.lambda_)

    def compute_output(self, _):
        if not self.active or not self.goal_set() or self.reached_goal:
            return

        # State descriptive variables
        r, theta, delta, virtual_control_delta, K = self._get_descriptive_state_variables()
        v = self._dynamic_velocity(K)
        w = - v / r * (self.k2 * (delta - virtual_control_delta) + (1 + self.k1 / (1 + (self.k1 * theta)**2)) * np.sin(theta))

        if r < self.r_tolerance:
            rospy.logwarn(f'Goal reached')
            self.reset()
            rospy.logwarn(f'Pose controller deactivated')
            self.active = False
            self.reached_goal_publisher.publish(Bool(True))
            v = 0.0
            w = 0.0
        
        twist_msg = Twist( linear = Vector3(x = v, y = 0.0, z = 0.0),
                            angular = Vector3(x = 0.0, y = 0.0, z = w))
                
        self.cmd_vel_publisher.publish(twist_msg)
        self.error_publisher.publish(Float32(r))


if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("smooth_controller")

    # Get Global ROS parameters
    params = get_global_params()
    
    # Get Local ROS parameters
    k1 = rospy.get_param('~k1', 0.0)
    k2 = rospy.get_param('~k2', 0.0)
    beta = rospy.get_param('~beta', 0.0)
    lambda_ = rospy.get_param('~lambda', 0.0)
    v_max = rospy.get_param('~v_max', 0.0)
    r_tolerance = rospy.get_param('~r_tolerance', 0.0)
    odom_topic = rospy.get_param('~odom_topic', '/puzzlebot/gazebo_odom')

    # Controller instance
    starting_pose = Point(x = params['starting_state']['x'], 
                          y = params['starting_state']['y'], 
                          z = 0.0)
    starting_orientation = params['starting_state']['theta']

    # Initialize controller
    puzzlebot_controller = Puzzlebot_controller(starting_pose = starting_pose,
                                                starting_orientation = starting_orientation,
                                                k1 = k1, k2 = k2, beta = beta, lambda_ = lambda_, v_max = v_max,
                                                r_tolerance = r_tolerance,
                                                commands_topic = params['commands_topic'],
                                                odom_topic = odom_topic,
                                                goals_topic = params['pose_controller_topic'],
                                                send_done_topic = params['unlock_topic'],
                                                pose_control_activate_topic = rospy.get_param('/pose_control_activate_topic'),
                                                control_rate = params['control_rate'])
    
    try:
        rospy.loginfo('The controller node is Running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
