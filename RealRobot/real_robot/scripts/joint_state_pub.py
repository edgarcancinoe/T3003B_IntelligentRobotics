#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform, PoseWithCovariance, TwistWithCovariance, Pose, Point, Twist
from puzzlebot_util.util import *
import tf.transformations as tft

class Joint_State_Publisher():
    """
        Listen to odometry topic and publish to rviz
    """
    def __init__(self, odom_frame, map_frame, robot_frame_name, r, l,
                 joint_names, joint_initial_positions, joint_states_topic, 
                 starting_state, odom_topic, map_topic, rviz_rate):
        
        # Save simulation joint parameters
        self.joint_names = joint_names
        self.joints_positions = np.array(joint_initial_positions)
        self.r = r
        self.l = l
        
        # Message configurations
        self.odom_frame = odom_frame
        self.map_frame = map_frame
        self.robot_body_id = robot_frame_name

        self.effort_constant = [0.0]*len(self.joint_names)
        
        # TF broadcaster to establish frame relations
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.set_odom_frame()

        # Callback messages
        self.odom_msg = Odometry()
        self.odom_msg.pose.pose.orientation.w = 1

        self.map_msg = Odometry()
        self.map_msg.pose.pose.orientation.w = 1
        
        # Suscsribe to Odom topic
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(map_topic, Odometry, self.map_callback)

        # Joint State Publisher to send joint states to rviz simualtion
        self.joint_state_publisher = rospy.Publisher(joint_states_topic, JointState, queue_size=10)
        
        self.listening_odom = False
        self.listening_map = False
        self.last_timestamp = rospy.Time.now()

        rospy.Timer(rospy.Duration(1.0/rviz_rate), self.update)

    def set_odom_frame(self):
        odom_transform = TransformStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=self.odom_frame),
            child_frame_id=self.robot_body_id,
            transform=Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0, 0, 0, 1) 
            )
        )
        # Broadcast the static transform
        self.tf_broadcaster.sendTransform(odom_transform)

        odom_transform = TransformStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=self.map_frame),
            child_frame_id=self.robot_body_id,
            transform=Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0, 0, 0, 1) 
            )
        )
        # Broadcast the static transform
        self.tf_broadcaster.sendTransform(odom_transform)

    def odom_callback(self, msg):
        if self.listening_odom:
            self.odom_msg = msg
            self.listening_odom = False

    def map_callback(self, msg):
        if self.listening_map:
            self.map_msg = msg
            self.listening_map = False

    def broadcast_robot_transform(self, position, orientation, frame, stamp):
        # Prepare the transformation object with all required fields
        robot_transform = TransformStamped(
            header=Header(stamp=stamp, frame_id=frame),
            child_frame_id=self.robot_body_id,
            transform=Transform(
                translation=Vector3(x=position.x, y=position.y, z=position.z),
                rotation=orientation
            )
        )
        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(robot_transform)
    
    def _publish_sim(self):
        
        stamp = rospy.Time.now()
        
        # 1.
        robot_position_inertial = self.odom_msg.pose.pose.position
        robot_orientation_inertial = self.odom_msg.pose.pose.orientation

        robot_position_map = self.map_msg.pose.pose.position
        robot_orientation_map = self.map_msg.pose.pose.orientation
        
        self.broadcast_robot_transform(robot_position_inertial, robot_orientation_inertial, self.odom_frame, stamp)
        self.broadcast_robot_transform(robot_position_map, robot_orientation_map, self.map_frame, stamp)

        
    def update(self, _):
        self._publish_sim()
        self.listening_odom = True
        self.listening_map = True

if __name__ == '__main__':
    rospy.init_node('joint_state_pub')

    # Get Global ROS parameters
    params = get_global_params()
    # Get Local ROS parameters
    joint_names = rospy.get_param('~joint_names', ['leftWheel', 'rightWheel'])
    joint_initial_positions = rospy.get_param('~joint_initial_positions', [0.0, 0.0])
    joint_states_topic = rospy.get_param('~joint_states', '/joint_states')
    robot_frame = rospy.get_param('/robot_frame')
    odom_frame = rospy.get_param('/odom_frame')
    map_frame = rospy.get_param('/map_frame')
    map_topic = rospy.get_param('/map_topic')
    
    joint_state_publisher = Joint_State_Publisher(odom_frame=odom_frame, 
                                                  map_frame = map_frame,
                                                  robot_frame_name=robot_frame,
                                                  r=params['wheel_radius'], l=params['track_length'],
                                                  joint_names=joint_names, 
                                                  joint_initial_positions=joint_initial_positions, 
                                                  joint_states_topic=joint_states_topic,
                                                  starting_state = params['starting_state'],
                                                  odom_topic = params['odometry_topic'],
                                                  map_topic = map_topic,
                                                  rviz_rate = params['rviz_rate'])

    try:
        rospy.loginfo('Joint State node running')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
