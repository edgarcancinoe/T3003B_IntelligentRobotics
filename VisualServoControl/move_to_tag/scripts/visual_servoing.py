#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Vector3Stamped, Twist, Quaternion, QuaternionStamped, Polygon, Point
import tf
import tf.transformations as tft
import math
class Puzzlebot_Visual_Controller():
    def __init__(self, kp, desired_corner_locations, z_desired, focal_length_pixel,
                 error_tolerance, commands_topic, input_topic, control_rate):

        # Proportional controller gains
        self.kp = kp

        # Reference corner locations (format [u1 v1, u2 v2, u3 v3, u4 v4])
        self.p_desired = desired_corner_locations.flatten()
        self.z_desired = z_desired

        # State (format [u1 v1, u2 v2, u3 v3, u4 v4])
        self.p: np.array = None
        self.z_values = [0.0, 0.0, 0.0, 0.0]

        # Camera parameters
        self.focal_length_pixel = focal_length_pixel

        # Set tolerances
        self.error_tolerance = error_tolerance

        # Publishers
        self.cmd_vel_publisher = rospy.Publisher(commands_topic, Twist, queue_size=10)

        # Suscribers
        rospy.Subscriber(input_topic, Polygon, self.vision_callback)
        self.tf_listener = tf.TransformListener()
        
        rospy.Timer(rospy.Duration(1.0/control_rate), self.compute_output)    
        self.no_target_on_sight = True
    
    def _compute_errors(self) -> np.ndarray: # Should return a nx1 array n = 8
        # Compute errors
        e = self.p_desired - self.p
        return e
    
    def _jp_matrix(self, v, u, z):
        z = (z + self.z_desired)/2
        j_p = np.array([[u/z, -(self.focal_length_pixel+u**2/self.focal_length_pixel)],
                        [v/z, -u*v/self.focal_length_pixel]])
        return j_p
    
    def _inv_jacobian_matrix(self, u, v, z):
        J = np.concatenate([self._jp_matrix(v[0], u[0], z[0]),
                            self._jp_matrix(v[1], u[1], z[1]),
                            self._jp_matrix(v[2], u[2], z[2]),
                            self._jp_matrix(v[3], u[3], z[3])], axis=0)
        
        return np.linalg.pinv(J)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    def _quaternion_to_euler(self, x, y, z, w):

        # Compute roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Compute pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Compute yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        print((roll, pitch, yaw))
        return (roll, pitch, yaw)

    def vision_callback(self, msg):
        assert len(msg.points) == 4, 'The polygon must have 4 points'
        # Format [u1 v1, u2 v2, u3 v3, u4 v4]
        self.p = np.array([[point.x, point.y] for point in msg.points]).flatten()
        self.z_values = [point.z for point in msg.points]

    def compute_output(self, _):
        
        # Control output initialization
        twist_msg = Twist( linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                            angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
        
        # Proportional error
        flag = False
        if self.p is not None:
            e = self._compute_errors()
            print(np.linalg.norm(e))
            if np.linalg.norm(e) <= self.error_tolerance:
                rospy.logwarn('Goal reached')
            else:
                # Compute cameras desired velocity for state s: [v*_x, v*_y, w*] (CAMERA FRAME)
                u = [self.p[0], self.p[2], self.p[4], self.p[6]]
                v = [self.p[1], self.p[3], self.p[5], self.p[7]]

                s_dot = self.kp * self._inv_jacobian_matrix(u, v, self.z_values) @ e # 2x1 array [v_z, w_y] in camera frame
                
                # Compute the desired linear and angular velocities in the robot frame
                """
                header = Header(stamp=rospy.Time(0), frame_id='/camera_link')
                
                # Express rotation as quaternion
                quat = self.euler_to_quaternion(0.0, s_dot[1], 0.0)
                position = [0.0, 0.0, s_dot[0]]

                # Get transformation matrix
                transformMatrix = self.tf_listener.asMatrix('/base_link', header)

                # Compute the pose as a matrix
                position_as_matrix = tft.translation_matrix(position)
                rotation_as_matrix = tft.quaternion_matrix(quat)
                pose_as_matrix = np.dot(position_as_matrix, rotation_as_matrix)
                pose_target_frame = np.dot(transformMatrix, pose_as_matrix)

                pos_target_frame = Point(*tuple(tft.translation_from_matrix(pose_target_frame))[:3])
                ortn_target_frame = Quaternion(*tuple(tft.quaternion_from_matrix(pose_target_frame)))
              
                # Compute the desired linear and angular velocities 
                # twist_msg.linear.x = s_dot[0]
                # twist_msg.angular.z = -self._quaternion_to_euler(ortn_target_frame.x, ortn_target_frame.y, ortn_target_frame.z, ortn_target_frame.w)[2]
                """
                twist_msg.linear.x = s_dot[0]
                twist_msg.angular.z = -s_dot[1]*1.8
                
                self.no_target_on_sight = False
                flag = True
            print()
            
        
        if not flag and not self.no_target_on_sight:
            self.cmd_vel_publisher.publish(twist_msg)
            self.no_target_on_sight = True
        elif flag:
            self.cmd_vel_publisher.publish(twist_msg)
        # Reset state
        self.p = None


if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("visual_servoing")

    # Get Global ROS parameters
    input_topic = rospy.get_param('/visual_controller_topic')
    commands_topic = rospy.get_param('~commands_topic')
    z_desired = rospy.get_param('/z_desired')
    focal_length_pixel = rospy.get_param('/focal_length_pixel')
    desired_corner_locations = np.array([rospy.get_param('/desired_corner_locations')])
    control_rate = rospy.get_param('/control_rate')

    kp = rospy.get_param('~kp')
    e_tolerance = rospy.get_param('~e_tolerance')

    # Initialize controller
    puzzlebot_controller = Puzzlebot_Visual_Controller(kp=kp, 
                                                        desired_corner_locations = desired_corner_locations,
                                                        z_desired=z_desired,
                                                        focal_length_pixel=focal_length_pixel,
                                                        error_tolerance = e_tolerance, 
                                                        commands_topic = commands_topic, 
                                                        input_topic = input_topic, 
                                                        control_rate = control_rate)
    
    try:
        rospy.loginfo('The controller node is Running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

