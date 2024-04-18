#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3, Twist, Point, Pose, Quaternion
from nav_msgs.msg import Odometry
from puzzlebot_util.util import get_global_params

class Puzzlebot_controller():
    def __init__(self, starting_pose, starting_orientation,
                 kp_V, ki_V, kd_V, kp_w, ki_w, kd_w, 
                 d_tolerance, deg_tolerance, commands_topic):

        # Proportional controller gains
        self.kp_l = kp_V
        self.kp_w = kp_w
        # Integral controller gains
        self.ki_l = ki_V
        self.ki_w = ki_w
        # Derivative controller gains
        self.kd_l = kd_V
        self.kd_w = kd_w

        # Reference (goal)
        self.s_d : Point = None

        # Initial state (inertial frame)
        self.s : Pose = Pose(position=starting_pose, 
                             orientation=Quaternion(x=0.0, y=0.0, z=starting_orientation, w=1.0))

        # Set tolerances (and convert deg to radians)
        self.w_tolerance : float = (deg_tolerance*(1/360)) * (2 * np.pi) 
        self.d_tolerance : float = d_tolerance
        
        # Publishers
        self.cmd_vel_publisher = rospy.Publisher('/' + commands_topic, Twist, queue_size=10)
        
    def _compute_errors(self):
        e_x = self.s_d.x - self.s.position.x
        e_y = self.s_d.y - self.s.position.y
        e_l = np.sqrt((e_x * e_x) + (e_y * e_y))
        w_d = np.arctan2(e_y, e_x)
        e_w = w_d - self.s.orientation.z
        return e_l, e_w
    
    def odometry_callback(self, msg):
        self.s = msg.pose.pose        
    
    def set_goal(self, goal: Point):
        self.s_d = goal

    def goal_set(self):
        return self.s_d == None

    def compute_output(self):
        e_l, e_w, = self._compute_errors()
        twist_msg = Twist( linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                            angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
        
        if abs(e_w) > self.w_tolerance:
            twist_msg.angular.z = self.kp_w * e_w
        elif abs(e_l) > self.d_tolerance:
            twist_msg.linear.x = self.kp_l * e_l
        else:
            return True
        self.cmd_vel_publisher.publish(twist_msg)
        return False
    

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

    # List of goals to reach
    goals = [Point(x = 1.0, y = 0.0, z = 0.0),
             Point(x = 1.0, y = 1.0, z = 0.0),
             Point(x = 0.0, y = 1.0, z = 0.0),
             Point(x = 0.0, y = 0.0, z = 0.0)]

    # List index
    current_goal = 0

    # Initialize controller
    puzzlebot_controller = Puzzlebot_controller(starting_pose, starting_orientation, 
                                                kp_V, ki_V, kd_V, kp_w, ki_w, kd_w,
                                                d_tolerance, deg_tolerance, params['commands_topic'])


    rospy.Subscriber(params['odometry_topic'], Odometry, puzzlebot_controller.odometry_callback)
    

    loop_rate = rospy.Rate(params['freq'])
    rospy.loginfo('The controller node is Running')
    
    # Set goal
    puzzlebot_controller.set_goal(goals[current_goal])
    
    print(f'Reaching for goal {current_goal}..')
    try:
        while not rospy.is_shutdown():
            reached_goal = puzzlebot_controller.compute_output()
            
            if reached_goal:
                print(f'Goal {current_goal} reached')
                current_goal += 1
                current_goal = current_goal % 4
                print(f'Reaching for goal {current_goal}..')
                puzzlebot_controller.set_goal(goals[current_goal])
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass 