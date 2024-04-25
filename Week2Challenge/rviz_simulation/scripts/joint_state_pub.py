#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform, PoseWithCovariance, TwistWithCovariance, Pose, Point, Twist
from puzzlebot_util.util import *

class Joint_State_Publisher():
    def __init__(self, inertial_frame_name, robot_frame_name, 
                 joint_names, joint_initial_positions, joint_states_topic, 
                 starting_state, odom_topic, rviz_rate):
        # Save simulation joint parameters
        self.joint_names = joint_names
        self.joints_positions = np.array(joint_initial_positions)
        
        # Message configurations
        self.frame_id = inertial_frame_name
        self.robot_body_id = robot_frame_name
        self.effort_constant = [0.0]*len(self.joint_names)
        
        # TF broadcaster to establish frame relations
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.set_odom_frame()

        # Callback messages
        self.odom_msg = Odometry(
            header = Header(frame_id = self.frame_id, stamp = rospy.Time.now()),
            child_frame_id = '',
            # Pose in inertial frame (world_frame)
            pose = PoseWithCovariance(
                pose = Pose(
                    position = Point(x = starting_state['x'], y = starting_state['y'], z = 0.0),
                    orientation = Quaternion(x = 0.0, y = 0.0, z = wrap_to_Pi(starting_state['theta']), w = 1.0)
                ),
                covariance = None
            ),
            # Twist in child frame (puzzlebot)
            twist = TwistWithCovariance(
                twist = Twist(
                    linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = 0.0)
                ),
                covariance = None
            )
        )
        
        # Suscsribe to Odom topic
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        # Joint State Publisher to send joint states to rviz simualtion
        self.joint_state_publisher = rospy.Publisher(joint_states_topic, JointState, queue_size=10)
        
        self.listening = False
        self.last_timestamp = rospy.Time.now()

        rospy.Timer(rospy.Duration(1.0/rviz_rate), self.talk_to_rviz)

    def _get_dt(self):
        dt = (self.odom_msg.header.stamp - self.last_timestamp).to_sec()
        self.last_timestamp = self.odom_msg.header.stamp
        return dt
    
    def _integrate_velocity(self, wdot, dt):
        return wdot * dt
    
    def _quaternion_from_z_rotation(self, yaw):
        """
            Convert an euler z-axis rotation (radians) to quaternion form
        """
        w = np.cos(yaw / 2)
        z = np.sin(yaw / 2)
        
        return (0, 0, z, w)
    
    def set_odom_frame(self):
        odom_transform = TransformStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=self.frame_id),
            child_frame_id=self.robot_body_id,
            transform=Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0, 0, 0, 1) 
            )
        )
        # Broadcast the static transform
        self.tf_broadcaster.sendTransform(odom_transform)

    def odom_callback(self, msg):
        if self.listening:
            self.odom_msg = msg
            self.listening = False

    def broadcast_robot_transform(self, position, orientation, stamp):
        # Prepare the transformation object with all required fields
        robot_transform = TransformStamped(
            header=Header(stamp=stamp, frame_id=self.frame_id),
            child_frame_id=self.robot_body_id,
            transform=Transform(
                translation=Vector3(x=position.x, y=position.y, z=position.z),
                rotation=Quaternion(*self._quaternion_from_z_rotation(orientation))
            )
        )
        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(robot_transform)
    
    def _publish_sim(self, dt):
        # The odometry message contains:
        # a. Pose: Position and orientation of the robot in inertial frame
        # b. Twist: linear and angular velocities of the robot wheels in the robot frame

        # What we must do with this information:
        # 1. Public dynamic tf based on new position of the robot so that we can always transform relative to the base, inertial frame
        # 2. Transform the values of joints given by the odometry transformed (publish in /joint_states).
        stamp = rospy.Time.now()
        # 1.
        robot_position_inertial = self.odom_msg.pose.pose.position
        robot_orientation_inertial = self.odom_msg.pose.pose.orientation.z
        self.broadcast_robot_transform(robot_position_inertial, robot_orientation_inertial, stamp)

        # 2.
        joints_velocities = np.array([self.odom_msg.twist.twist.angular.x, 
                                      self.odom_msg.twist.twist.angular.y])
        self.joints_positions = wrap_to_Pi(self.joints_positions + self._integrate_velocity(joints_velocities, dt))
        
        self.joint_state_publisher.publish(JointState(header=Header(frame_id = self.robot_body_id, stamp = stamp), 
                                                      position=self.joints_positions, 
                                                      velocity=joints_velocities, 
                                                      name=self.joint_names, 
                                                      effort=self.effort_constant))
        
    def talk_to_rviz(self, _):
        dt = self._get_dt()
        self._publish_sim(dt)
        self.listening = True

if __name__ == '__main__':
    rospy.init_node('joint_state_pub')

    # Get Global ROS parameters
    params = get_global_params()
    # Get Local ROS parameters
    joint_names = rospy.get_param('~joint_names', ['leftWheel', 'rightWheel'])
    joint_initial_positions = rospy.get_param('~joint_initial_positions', [0.0, 0.0])
    joint_states_topic = rospy.get_param('~joint_states', '/joint_states')
    
    joint_state_publisher = Joint_State_Publisher(inertial_frame_name=params['inertial_frame_name'], 
                                                  robot_frame_name=params['robot_frame_name'],
                                                  joint_names=joint_names, 
                                                  joint_initial_positions=joint_initial_positions, 
                                                  joint_states_topic=joint_states_topic,
                                                  starting_state = params['starting_state'],
                                                  odom_topic = params['odometry_topic'],
                                                  rviz_rate = params['rviz_rate'])

    try:
        rospy.loginfo('Joint State node running')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
