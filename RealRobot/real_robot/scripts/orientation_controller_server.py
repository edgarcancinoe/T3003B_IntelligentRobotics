#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Vector3, Twist, Point
from nav_msgs.msg import Odometry
from real_robot.srv import OrientationService, OrientationServiceResponse
from real_robot_util.util import get_orientation_controller_params
import tf.transformations as tf

class Puzzlebot_orientation_controller():
    def __init__(self, active, goal, starting_orientation,
                 kp, ki, e_tolerance, 
                 commands_topic, odom_topic, 
                 control_rate, max_w, min_w, min_w_to_move):

        # Control parameter sof the wheels to ensure they both move and avoid drifting
        self.kp = kp
        self.ki = ki
        self.ei = 0.0
        self.max_w = max_w
        self.min_w = min_w
        self.min_w_to_move = min_w_to_move
        # Reference (goal)
        self.target_yaw : float = goal

        # Initial state (inertial frame)
        self.yaw : float = starting_orientation

        # Set tolerances 
        self.e_tolerance : float = e_tolerance
        
        # Publishers
        self.cmd_vel_publisher = rospy.Publisher(commands_topic, Twist, queue_size=10)
        #self.reached_goal_publisher = rospy.Publisher(send_done_topic, Bool, queue_size=10)

        # Suscriber
        rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)
        #rospy.Subscriber(goals_topic, Float32, self.set_goal)
        #rospy.Subscriber(orientation_control_activate_topic, Bool, self._activate_controller)

        # Useful variables        
        self.reached_goal = False
        self.starting_yaw = None # To check if the orientation has crossed the 0 angle while turning
        self.active = active

        #Init Goal
        rospy.logwarn(f'\nNew orientation goal:\n {self.target_yaw}')
        while self.starting_yaw == None:
            rospy.loginfo(f'Waiting for odometry orientation')
            self.starting_yaw = self.yaw

        self.time = rospy.Time.now()
        rospy.Timer(rospy.Duration(1.0/control_rate), self.compute_output)
        
    #def _activate_controller(self, msg):
    #    if msg.data:
    #        rospy.logwarn(f'Orientation controller activated')
    #    else:
    #        rospy.logwarn(f'Orientation controller deactivated')
    #    self.active = msg.data

    def _quat2yaw(self, quat):
        euler = tf.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        yaw = euler[2]
        return yaw
    
    def odometry_callback(self, msg):
        self.yaw = self._quat2yaw(msg.pose.pose.orientation)

    def reset(self):
        self.ei = 0.0
        self.yaw = None
        self.target_yaw = None
        self.active = False
        self.starting_yaw = None
        self.reached_goal = False
        rospy.logwarn(f'Orientation controller deactivated')

    #def set_goal(self, goal: Float32):
    #    self.ei = 0.0
    #    rospy.logwarn(f'\nNew orientation goal:\n {goal.data}')
    #    self.target_yaw = goal.data
    #    while self.starting_yaw == None:
    #        rospy.loginfo(f'Waiting for odometry orientation')
    #        self.starting_yaw = self.yaw

    #def goal_set(self):
    #    return self.target_yaw != None
    
    def _get_dt(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.time).to_sec()
        self.time = current_time
        return dt
    
    def compute_output(self, _):
        dt = self._get_dt()
        if not self.active:
        #if not self.active or not self.goal_set() or self.reached_goal or self.starting_yaw == None:
            return
        target = self.target_yaw

        yaw = (self.yaw + 2*np.pi) % (2*np.pi) # Ensure range [0, 2*pi]
        
        # e = (target - yaw) if yaw < self.starting_yaw else (target - yaw) % (2*np.pi)
        e = ((target - yaw) + np.pi ) % (2*np.pi) - np.pi # Ensure range [-pi, pi]

        self.ei += e * dt
        if abs(e) < self.e_tolerance:
            rospy.logwarn(f'Goal reached')
            #self.reached_goal_publisher.publish(Bool(True))
            self.reset()
            w = 0.0
            self.reached_goal = True
            twist_msg = Twist( linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = w))
                
            self.cmd_vel_publisher.publish(twist_msg)
            return
        else:
            w = self.kp * e + self.ki * self.ei

            # Sanity check on the angular velocity
            if abs(w) < self.min_w_to_move:
                w = 0.0
                rospy.logwarn(f'Goal reached (small w)')
                #self.reached_goal_publisher.publish(Bool(True))
                self.reset()
                self.reached_goal = True
                w = 0.0
                twist_msg = Twist( linear = Vector3(x = 0.0, y = 0.0, z = 0.0),angular = Vector3(x = 0.0, y = 0.0, z = w))
                self.cmd_vel_publisher.publish(twist_msg)
                return
            elif abs(w) < self.min_w:
                w = np.sign(w) * self.min_w
            else:
                w = np.clip(w, -self.max_w, self.max_w)

        twist_msg = Twist( linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                            angular = Vector3(x = 0.0, y = 0.0, z = w))
                
        self.cmd_vel_publisher.publish(twist_msg)

def handle_orientation(req):
    # Get Global ROS parameters
    params = get_orientation_controller_params()
    
    # Get Local ROS parameters
    kp = params['kp']
    ki = params['ki']
    e_tolerance = params['e_tolerance']
    odom_topic = params['odometry_topic']
    #goals_topic =  rospy.get_param('/orientation_controller_topic')
    #orientation_control_activate_topic = rospy.get_param('/orientation_control_activate_topic')

    min_w = params['min_w']
    min_w_to_move = params['min_w_to_move']

    max_w = params['max_w']

    # Controller instance
    #starting_pose = Point(x = params['starting_state']['x'], 
    #                      y = params['starting_state']['y'], 
    #                      z = 0.0)
    starting_orientation = params['starting_state']['theta']
    rospy.loginfo("Service called, computation started.")
    active = True
    #reached_goal = False
    # Initialize controller
    puzzlebot_controller = Puzzlebot_orientation_controller(
                                                active = active,
                                                goal = req.goal.data,
                                                starting_orientation = starting_orientation,
                                                kp =  kp,
                                                ki = ki,
                                                e_tolerance = e_tolerance,
                                                commands_topic = params['commands_topic'],
                                                odom_topic = odom_topic,
                                                #goals_topic = goals_topic,
                                                #send_done_topic = params['unlock_topic'],
                                                #orientation_control_activate_topic = orientation_control_activate_topic,
                                                control_rate = params['control_rate'],
                                                max_w = max_w,
                                                min_w = min_w,
                                                min_w_to_move = min_w_to_move)
    # Wait for the goal to be reached
    while not rospy.is_shutdown() and not puzzlebot_controller.reached_goal:
        rospy.sleep(0.1)
    
    rospy.loginfo("Goal reached, service finished.")
    active = False
    return OrientationServiceResponse(True)

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("orientation_server", anonymous = True)
    service = rospy.Service('orientation_controller', OrientationService, handle_orientation)
    rospy.loginfo("Orientation Service is ready")
    
    rospy.sleep(3)

    rospy.spin()