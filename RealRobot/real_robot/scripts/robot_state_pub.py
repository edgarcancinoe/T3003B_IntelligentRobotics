#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform, PoseWithCovariance, TwistWithCovariance, Pose, Point, Twist
from real_robot_util.util import get_robot_state_pub_params
import tf.transformations as tft

class Robot_State_Publisher():
    """
        Listen to odometry topic and publish transforms
    """
    def __init__(self, odom_frame, robot_frame_name,
                 starting_state, odom_topic, update_rate):
                
        # Message configurations
        self.odom_frame = odom_frame
        self.robot_body_id = robot_frame_name
        
        # TF broadcaster to establish frame relations
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.set_odom_frame()

        # Callback messages
        self.odom_msg = Odometry()
        self.odom_msg.pose.pose.orientation.w = 1
        
        # Suscsribe to Odom topic
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        
        self.listening_odom = False
        self.last_timestamp = rospy.Time.now()

        rospy.Timer(rospy.Duration(1.0/update_rate), self.update)

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

    def odom_callback(self, msg):
        # print('Received odometry message')
        if self.listening_odom:
            # print(msg)
            self.odom_msg = msg
            self.listening_odom = False

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

        self.broadcast_robot_transform(robot_position_inertial, robot_orientation_inertial, self.odom_frame, stamp)

        
    def update(self, _):
        self._publish_sim()
        self.listening_odom = True

if __name__ == '__main__':
    rospy.init_node('robot_state_pub')

    # Get ROS parameters
    params = get_robot_state_pub_params()

    robot_state_publisher = Robot_State_Publisher(odom_frame=params['inertial_frame'], 
                                                  robot_frame_name=params['robot_frame'],
                                                  starting_state = params['starting_state'],
                                                  odom_topic = params['odometry_topic'],
                                                  update_rate = params['update_rate'])

    try:
        rospy.loginfo('Robot state publisher node running')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        