#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from puzzlebot_util.util import *
import tf.transformations as tft

class Puzzlebot_controller():
    def __init__(self, starting_pose, starting_orientation,
                 kp_V, ki_V, kd_V, kp_w, ki_w, kd_w, 
                 d_tolerance, deg_tolerance, commands_topic, odom_topic, control_rate):

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

        # Set tolerances (and convert deg to radians)
        self.w_tolerance : float = (deg_tolerance*(1/360)) * (2 * np.pi) 
        self.d_tolerance : float = d_tolerance
        
        # Publishers
        rospy.logwarn('Publishing to ' + commands_topic + ' topic for commands (kinematic model)')
        self.cmd_vel_publisher = rospy.Publisher('/' + commands_topic, Twist, queue_size=10)
        self.error_publisher = rospy.Publisher('/errors', Twist, queue_size=10)
        self.ref_publisher = rospy.Publisher('/ref', Twist, queue_size=10)
        # Suscriber
        rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)
        
        
        # List of goals to reach
        self.goals = [
                Point(x = 0.385, y = 0.00, z = 0.00),
                Point(x = 0.192, y = 0.333, z = 0.00),
                Point(x = -0.192, y = 0.333, z = 0.00),
                Point(x = -0.385, y = 0.00, z = 0.00),
                Point(x = -0.192, y = -0.333, z = 0.00),
                Point(x = 0.192, y = -0.333, z = 0.00)
                ]

        # List index
        self.current_goal = 0
        self.n_goals = len(self.goals)

        # Set goal
        self.set_goal(self.goals[self.current_goal])
        
        self.reached_goal = False

        rospy.Timer(rospy.Duration(1.0/control_rate), self.compute_output)

        # Time
        self.time = rospy.Time.now()



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
    
    def set_goal(self, goal: Point):
        self.ei_l = 0.0
        self.ei_w = 0.0
        self.e_l_prev = 0.0
        self.e_w_prev = 0.0
        self.s_d = goal
        rospy.logwarn(f'Reaching for goal {self.current_goal}..')

    def goal_set(self):
        return self.s_d == None
    
    def _compute_errors(self):
        # Position errors
        e_x = self.s_d.x - self.s.position.x
        e_y = self.s_d.y - self.s.position.y

        # Distance error
        e_l = np.sqrt((e_x * e_x) + (e_y * e_y))
        
        # Angular reference
        w_d = np.arctan2(e_y, e_x)
        # Angular errror
        e_w = w_d - tft.euler_from_quaternion([self.s.orientation.x, self.s.orientation.y, self.s.orientation.z, self.s.orientation.w])[2]
        
        if np.abs(e_l) < self.d_tolerance:
            e_l = 0.0
        if np.abs(e_w) < self.w_tolerance:
            e_w = 0.0

        ref = Twist( linear = Vector3(x = self.s_d.x, y = self.s_d.y, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = w_d))
        
        self.ref_publisher.publish(ref)
        return e_l, e_w
    
    def compute_output(self, _):
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
        elif e_l != 0.0: # If aligned
            twist_msg.linear.x = (self.kp_l * e_l) + (self.ki_l * self.ei_l) + (self.kd_l * e_dot_l)
        else: # Reached Goal
            rospy.logwarn(f'Goal {self.current_goal} reached')
            self.current_goal += 1
            self.current_goal = self.current_goal % self.n_goals
            self.set_goal(self.goals[self.current_goal])

        self.cmd_vel_publisher.publish(twist_msg)
        self.error_publisher.publish(errors)



if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("control")

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
    deg_tolerance = rospy.get_param('~deg_tolerance', 0.0)

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
                                                deg_tolerance = deg_tolerance, 
                                                commands_topic = params['commands_topic'],
                                                odom_topic = params['odometry_topic'],
                                                control_rate = params['control_rate'])
    try:
        rospy.loginfo('The controller node is Running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

