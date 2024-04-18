#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped

class Joint_State_Publisher():
    def __init__(self, joint_states_topic='/joint_states'):
        self.joint_state_publisher = rospy.Publisher(joint_states_topic, JointState, queue_size=10)
        self.joint_names = ['leftWheel', 'rightWheel']
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.time = rospy.Time.now()
        self.joints_positions = np.array([0.0, 0.0])
        self.frame_id = 'odom'
        self.robot_body_id = 'base_link'
        self.set_odom_frame()

    def _get_dt(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.time).to_sec()
        self.time = current_time
        return dt
    
    def _integrate_velocity(self, wdot, dt):
        return wdot * dt
    
    def _wrap_to_Pi(self, theta):
        result = np.fmod(theta + np.pi, 2 * np.pi)
        result[result < 0] += 2 * np.pi
        return result - np.pi
    
    def _euler_xyz_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (in radians) given in XYZ order to a quaternion.

        Parameters:
        - roll: Rotation around the X-axis in radians.
        - pitch: Rotation around the Y-axis in radians.
        - yaw: Rotation around the Z-axis in radians.

        Returns:
        - A tuple representing the quaternion (qx, qy, qz, qw).
        """
        # Half angles
        half_roll = roll / 2.0
        half_pitch = pitch / 2.0
        half_yaw = yaw / 2.0

        # Compute sines and cosines of half angles
        sin_r = np.sin(half_roll)
        cos_r = np.cos(half_roll)
        sin_p = np.sin(half_pitch)
        cos_p = np.cos(half_pitch)
        sin_y = np.sin(half_yaw)
        cos_y = np.cos(half_yaw)

        # Calculate the components of the quaternion
        qw = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y
        qx = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
        qy = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
        qz = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y

        return (qx, qy, qz, qw)
    
    def set_odom_frame(self):
        odom_transform = TransformStamped()
        odom_transform.header.stamp = rospy.Time.now()
        odom_transform.header.frame_id = self.frame_id
        odom_transform.child_frame_id = self.robot_body_id
        odom_transform.transform.translation.x = 0.0
        odom_transform.transform.translation.y = 0.0
        odom_transform.transform.translation.z = 0.0

        wx, wy, wz, ww = self._euler_xyz_to_quaternion(0,0,0)
        odom_transform.transform.rotation.x = wx
        odom_transform.transform.rotation.y = wy
        odom_transform.transform.rotation.z = wz
        odom_transform.transform.rotation.w = ww
        self.tf_broadcaster.sendTransform(odom_transform)

    def odom_callback(self, msg):
        # The odometry message contains:
        # a. Pose: Position and orientation of the robot in inertial frame
        # b. Twist: linear and angular velocities of the robot wheels in the robot frame

        # What we must do with this information:
        # 1. Public dynamic tf based on new position of the robot so that we can always transform relative to the base, inertial frame
        # 2. Transform the values of joints given by the odometry transformed (publish in /joint_states).

        # Define Header msg
        header = Header(frame_id = self.robot_body_id, stamp = rospy.Time.now())
        
        # 1.
        robot_position_inertial = msg.pose.pose.position
        robot_orientation_inertial = msg.pose.pose.orientation.z
        self.broadcast_robot_transform(robot_position_inertial, robot_orientation_inertial)

        # 2.
        joints_velocities = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y])
        self.joints_positions = self._wrap_to_Pi(self.joints_positions + self._integrate_velocity(joints_velocities, self._get_dt()))
        
        self.joint_state_publisher.publish(JointState(header=header, 
                                                      position=self.joints_positions, 
                                                      velocity=joints_velocities, 
                                                      name=self.joint_names, 
                                                      effort=[0.0]*len(self.joint_names)))
        
    
    def broadcast_robot_transform(self, position, orientation):
        robot_transform = TransformStamped()
        robot_transform.header.stamp = rospy.Time.now()
        robot_transform.header.frame_id = self.frame_id
        robot_transform.child_frame_id = self.robot_body_id
        robot_transform.transform.translation.x = position.x
        robot_transform.transform.translation.y = position.y 
        robot_transform.transform.translation.z = position.z
        wx, wy, wz, ww = self._euler_xyz_to_quaternion(0,0,orientation)
        robot_transform.transform.rotation.x = wx
        robot_transform.transform.rotation.y = wy
        robot_transform.transform.rotation.z = wz
        robot_transform.transform.rotation.w = ww
        self.tf_broadcaster.sendTransform(robot_transform)
    
    def step(self):
        return

freq = 100
if __name__ == '__main__':
    rospy.init_node('joint_state_pub')
    rate = rospy.Rate(rospy.get_param('~node_rate', freq))
    joint_state_publisher = Joint_State_Publisher()
    rospy.Subscriber('odom', Odometry, joint_state_publisher.odom_callback)
    rospy.loginfo('Joint State  node running')
    try:
        while not rospy.is_shutdown():
            joint_state_publisher.step()

    except rospy.ROSInterruptException:
        pass