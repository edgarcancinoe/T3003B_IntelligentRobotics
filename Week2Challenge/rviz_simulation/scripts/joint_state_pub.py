#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform
from puzzlebot_util.util import get_global_params

class Joint_State_Publisher():
    def __init__(self, inertial_frame_name, robot_frame_name, joint_names, joint_initial_positions, joint_states_topic):
        self.joint_state_publisher = rospy.Publisher(joint_states_topic, JointState, queue_size=10)
        self.joint_names = joint_names
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.time = rospy.Time.now()
        self.joints_positions = np.array(joint_initial_positions)
        self.frame_id = inertial_frame_name
        self.robot_body_id = robot_frame_name
        self.set_odom_frame()
        self.effort_constant = [0.0]*len(self.joint_names)

    def _get_dt(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.time).to_sec()
        self.time = current_time
        return dt
    
    def _integrate_velocity(self, wdot, dt):
        return wdot * dt
    
    def _wrap_to_Pi(self, theta):
        return theta
        result = np.fmod(theta + np.pi, 2 * np.pi)
        return result
        result[result < 0] += 2 * np.pi
        return result - np.pi
    
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
                                                      effort=self.effort_constant))
        
    
    def broadcast_robot_transform(self, position, orientation):
        # Prepare the transformation object with all required fields
        robot_transform = TransformStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=self.frame_id),
            child_frame_id=self.robot_body_id,
            transform=Transform(
                translation=Vector3(x=position.x, y=position.y, z=position.z),
                rotation=Quaternion(*self._quaternion_from_z_rotation(orientation))
            )
        )
        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(robot_transform)

    def step(self):
        return

if __name__ == '__main__':
    rospy.init_node('joint_state_pub')

    # Get Global ROS parameters
    params = get_global_params()
    # Get Local ROS parameters
    joint_names = rospy.get_param('~joint_names', ['leftWheel', 'rightWheel'])
    joint_initial_positions = rospy.get_param('~joint_initial_positions', [0.0, 0.0])
    joint_states_topic = rospy.get_param('~joint_states', '/joint_states')

    joint_state_publisher = Joint_State_Publisher(inertial_frame_name=params['inertial_frame_name'], robot_frame_name=params['robot_frame_name'],
                                                  joint_names=joint_names, 
                                                  joint_initial_positions=joint_initial_positions, 
                                                  joint_states_topic=joint_states_topic)

    rospy.Subscriber(params['odometry_topic'], Odometry, joint_state_publisher.odom_callback)

    loop_rate = rospy.Rate(params['freq'])
    rospy.loginfo('Joint State node running')
    try:
        while not rospy.is_shutdown():
            joint_state_publisher.step()
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass