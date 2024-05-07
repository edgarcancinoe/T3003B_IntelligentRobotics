#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from puzzlebot_util.util import *
import math

class Puzzlebot_controller():
    def __init__(self, starting_pose, starting_orientation,
                 kp_V, ki_V, kd_V, kp_w, ki_w, kd_w, 
                 d_tolerance, rad_tolerance, commands_topic, odom_topic, goals_topic, 
                 send_done_topic, pose_control_activate_topic, control_rate):

        # Proportional controller gains
        self.kp_l = kp_V
        self.kp_w = kp_w
        # Integral controller gains
        self.ki_l = ki_V
        self.ki_w = ki_w
        # Derivative controller gains
        self.kd_l = kd_V
        self.kd_w = kd_w

        # Cummulative errors
        self.ei_l = 0.0
        self.ei_w = 0.0
        
        # Last error
        self.e_l_prev = 0.0
        self.e_w_prev = 0.0

        # Reference (goal)
        self.s_d : Point = None

        # Initial state (inertial frame)
        self.s : Pose = Pose(position=starting_pose, 
                             orientation=Quaternion(x=0.0, y=0.0, z=starting_orientation, w=1.0))

        # Set tolerances 
        self.w_tolerance : float = rad_tolerance
        self.d_tolerance : float = d_tolerance
        
        # Publishers
        self.cmd_vel_publisher = rospy.Publisher(commands_topic, Twist, queue_size=10)
        self.error_publisher = rospy.Publisher('/errors', Twist, queue_size=10)
        self.ref_publisher = rospy.Publisher('/ref', Twist, queue_size=10)
        self.reached_goal_publisher = rospy.Publisher(send_done_topic, Bool, queue_size=10)

        # Suscriber
        rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)
        rospy.Subscriber(goals_topic, Point, self.set_goal)
        rospy.Subscriber(pose_control_activate_topic, Bool, self._activate_controller)

        # Goals        
        self.reached_goal = False
        self.aligned = False
        self.w_d = 0.0

        self.active = False

        rospy.Timer(rospy.Duration(1.0/control_rate), self.compute_output)

        # Time
        self.time = rospy.Time.now()

    def _activate_controller(self, msg):
        self.active = msg.data

    def _get_dt(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.time).to_sec()
        self.time = current_time
        return dt
    
    def _get_e_dot(self, e_l, e_w):
        e_dot_l = e_l - self.e_l_prev
        e_dot_w = e_w - self.e_w_prev
        self.e_l_prev = e_l
        self.e_w_prev = e_w
        return e_dot_l, e_dot_w

    def odometry_callback(self, msg):
        self.s = msg.pose.pose
    
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
        rospy.logwarn(f'\nReaching for goal:\n {self.s_d}\n')

    def goal_set(self):
        return self.s_d != None

    def _compute_errors(self):
        # Position errors
        e_x = self.s_d.x - self.s.position.x
        e_y = self.s_d.y - self.s.position.y
        # Distance error
        e_l = np.sqrt((e_x * e_x) + (e_y * e_y))
        
        # Angular reference
        w_d = np.arctan2(e_y, e_x)
        

        # s_w = self.s_orientation_as_euler()[2]
        
        # Angular errror
        # e_w = w_d - s_w
        
        e_w = w_d - self.s.orientation.z
        
        if np.abs(e_l) < self.d_tolerance:
            e_l = 0.0
        if np.abs(e_w) < self.w_tolerance:
            e_w = 0.0
            # self.aligned = True

        ref = Twist( linear = Vector3(x = self.s_d.x, y = self.s_d.y, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = w_d))

        self.ref_publisher.publish(ref)
        return e_l, e_w
    
    def compute_output(self, _):
        if not self.active or not self.goal_set():
            return

        # Proportional errors
        e_l, e_w, = self._compute_errors()
        
        dt = self._get_dt()

        # Integral error
        self.ei_l += e_l * dt
        self.ei_w += e_w * dt
        
        # Derivative error
        e_dot_l, e_dot_w = self._get_e_dot(e_l, e_w)

        errors = Twist( linear = Vector3(x = e_l, y = self.ei_l, z = e_dot_l),
                            angular = Vector3(x = e_w, y = self.ei_w, z = e_dot_w))
        
        twist_msg = Twist( linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                            angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
        
        # Not aligned
        if e_w != 0.0:
            twist_msg.angular.z = (self.kp_w * e_w) + (self.ki_w * self.ei_w) + (self.kd_w * e_dot_w)
        elif e_l != 0.0: # If aligned and not reached goal
            twist_msg.linear.x = (self.kp_l * e_l) + (self.ki_l * self.ei_l) + (self.kd_l * e_dot_l)
        else: # Reached Goal
            rospy.logwarn(f'Goal reached')
            self.reset()
            self.reached_goal_publisher.publish(Bool(True))

        self.cmd_vel_publisher.publish(twist_msg)
        self.error_publisher.publish(errors)


if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("pose_controller")

    # Get Global ROS parameters
    params = get_global_params()
    
    # Get Local ROS parameters
    kp_V = rospy.get_param('~kp_V', 0.0)
    ki_V = rospy.get_param('~ki_V', 0.0)
    kd_V = rospy.get_param('~kd_V', 0.0)

    kp_w = rospy.get_param('~kp_w', 0.0)
    ki_w = rospy.get_param('~ki_w', 0.0)
    kd_w = rospy.get_param('~kd_w', 0.0)

    d_tolerance = rospy.get_param('~d_tolerance', 0.0)
    rad_tolerance = rospy.get_param('~rad_tolerance', 0.0)

    # Controller instance
    starting_pose = Point(x = params['starting_state']['x'], 
                          y = params['starting_state']['y'], 
                          z = 0.0)
    starting_orientation = params['starting_state']['theta']

    # Initialize controller
    puzzlebot_controller = Puzzlebot_controller(starting_pose = starting_pose, 
                                                starting_orientation = starting_orientation, 
                                                kp_V = kp_V, ki_V = ki_V, kd_V = kd_V,
                                                kp_w = kp_w, ki_w = ki_w, kd_w = kd_w,
                                                d_tolerance = d_tolerance, 
                                                rad_tolerance = rad_tolerance, 
                                                commands_topic = params['commands_topic'],
                                                odom_topic = params['odometry_topic'],
                                                goals_topic = params['pose_controller_topic'],
                                                send_done_topic = params['unlock_topic'],
                                                pose_control_activate_topic = rospy.get_param('/pose_control_activate_topic'),
                                                control_rate = params['control_rate'])
    try:
        rospy.loginfo('The controller node is Running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

