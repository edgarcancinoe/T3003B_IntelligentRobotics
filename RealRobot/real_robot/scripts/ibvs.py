#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, Twist, Polygon
import tf2_ros
import tf.transformations as tft
import math
from real_robot_util.util import get_ibvs_params

class Puzzlebot_Visual_Controller():
    def __init__(self, kp, desired_corner_locations, z_desired, focal_length_pixel,
                 error_tolerance, commands_topic, input_topic, ibvs_activate_topic, 
                 send_done_topic, control_rate, robot_frame, camera_frame,max_w, min_w, min_w_to_move, max_v, min_v, min_v_to_move):

        # Proportional controller gains
        self.kp = kp

        # Limit velocities
        self.max_w = max_w
        self.min_w = min_w
        self.min_w_to_move = min_w_to_move
        self.min_v = min_v
        self.max_v = max_v
        self.min_v_to_move = min_v_to_move

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
        self.reached_goal_publisher = rospy.Publisher(send_done_topic, Bool, queue_size=10)

        # Suscribers
        rospy.Subscriber(input_topic, Polygon, self.vision_callback)
        rospy.Subscriber(ibvs_activate_topic, Bool, self._activate_controller)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.sleep(5.0)  # Wait for tf2 to initialize

        found = False
        while not found:
            try:
                trans = self.tf_buffer.lookup_transform(camera_frame, robot_frame, rospy.Time(0), rospy.Duration(4.0))
                tvec = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
                rvec = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
                R_r_c = tft.quaternion_matrix(rvec)[:3, :3]
            
                skew_symm_t = np.array([[0, -tvec[2], tvec[1]],
                                        [tvec[2], 0, -tvec[0]],
                                        [-tvec[1], tvec[0], 0]])  
                
                self.V_r_c_inv = np.linalg.inv(np.concatenate([np.concatenate([R_r_c, skew_symm_t @ R_r_c], axis=1),
                                            np.concatenate([np.zeros((3, 3)), R_r_c], axis=1)], axis=0))
                found = True
                rospy.loginfo('Transform found')
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Could not find the transformation. Retrying...")

        self.active = True

        rospy.Timer(rospy.Duration(1.0/control_rate), self.compute_output)    
        self.target_on_sight = False
    
    def _activate_controller(self, msg):
        self.active = msg.data
        if self.active:
            rospy.loginfo('IBVS controller activated.')

    def _compute_errors(self) -> np.ndarray:
        # Compute errors
        e = self.p_desired - self.p
        return e
    
    def _jp_matrix(self, v, u, z):
        z = (z + self.z_desired) / 2
        j_p = np.array([[u/z, -(self.focal_length_pixel + u**2 / self.focal_length_pixel)],
                        [v/z, -u * v / self.focal_length_pixel]])
        return j_p
    
    def _inv_jacobian_matrix(self, u, v, z):
        J = np.concatenate([self._jp_matrix(v[0], u[0], z[0]),
                            self._jp_matrix(v[1], u[1], z[1]),
                            self._jp_matrix(v[2], u[2], z[2]),
                            self._jp_matrix(v[3], u[3], z[3])], axis=0)
        
        return np.linalg.pinv(J)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

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

        return (roll, pitch, yaw)

    def vision_callback(self, msg):
        if len(msg.points) == 0:
            self.p = None
            self.target_on_sight = False
            return
        # Format [u1 v1, u2 v2, u3 v3, u4 v4]
        self.p = np.array([[point.x, point.y] for point in msg.points]).flatten()
        self.z_values = [point.z for point in msg.points]
        self.target_on_sight = True

    def compute_output(self, _):
        if not self.active:
            return
                
        # Proportional error
        done = False
        if self.target_on_sight:
            
            e = self._compute_errors()
            if np.linalg.norm(e) <= self.error_tolerance:
                rospy.logwarn('IBVS Goal reached')
                done = True
            else:
                # Compute cameras desired velocity for state s: [v*_x, v*_y, w*] (CAMERA FRAME)
                u = [self.p[0], self.p[2], self.p[4], self.p[6]]
                v = [self.p[1], self.p[3], self.p[5], self.p[7]]

                s_dot = self.kp * self._inv_jacobian_matrix(u, v, self.z_values) @ e  # 2x1 array [v_z, w_y] in camera frame
                
                # Compute the desired linear and angular velocities in the robot frame
                xi_dot_c = np.array([0.0, 0.0, s_dot[0], 0.0, s_dot[1], 0.0])
                xi_dot_r = self.V_r_c_inv @ xi_dot_c
                
                # Sanity check on the linear and angular velocities
                # Angular velocities
                if abs(xi_dot_r[5]) < self.min_w_to_move:
                    xi_dot_r[5] = 0.0
                elif abs(xi_dot_r[5]) < self.min_w:
                    xi_dot_r[5] = np.sign(xi_dot_r[5]) * self.min_w
                else:
                    xi_dot_r[5] = np.clip(xi_dot_r[5], -self.max_w, self.max_w)

                # Linear Velocities
                if abs(xi_dot_r[0]) < self.min_v_to_move:
                    xi_dot_r[0] = 0.0
                elif abs(xi_dot_r[0]) < self.min_v:
                    xi_dot_r[0] = np.sign(xi_dot_r[0]) * self.min_v
                else:
                    xi_dot_r[0] = np.clip(xi_dot_r[0], -self.max_v, self.max_v)


                twist_msg = Twist( linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                            angular = Vector3(x = 0.0, y = 0.0, z = 0.0))

                twist_msg.linear.x = xi_dot_r[0]
                twist_msg.angular.z = xi_dot_r[5] * 0.4

            if done:
                twist_msg = Twist( linear = Vector3(x = 0.0, y = 0.0, z = 0.0),
                                angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
                self.cmd_vel_publisher.publish(twist_msg)
                self.reached_goal_publisher.publish(done)
                self.active = False
                rospy.logwarn('DONE')
            elif not done:
                self.cmd_vel_publisher.publish(twist_msg)
        else: # No target on sight
            rospy.logwarn('Cannot see the target')
            self.reached_goal_publisher.publish(False)
            self.active = False


if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("ibvs_controller")
    
    params = get_ibvs_params()

    # Get ROS parameters
    input_topic = params['aruco_detection_topic']
    ibvs_activate_topic = params['ibvs_activate_topic']
    commands_topic = params['commands_topic']

    z_desired = params['target_z_desired']

    focal_length_pixel = params['focal_length_pixel']
    desired_corner_locations = params['target_corner_locations']

    control_rate = params['control_rate']

    robot_frame = params['robot_frame']
    camera_frame = params['camera_frame']

    kp = params['kp']
    e_tolerance = params['e_tolerance']

    min_w = params['min_w']
    min_w_to_move = params['min_w_to_move']
    max_w = params['max_w']
    max_v = params['max_v']
    min_v = params['min_v']
    min_v_to_move = params['min_v_to_move']

    send_done_topic = params['ibvs_done_topic']
    

    # Initialize controller
    puzzlebot_controller = Puzzlebot_Visual_Controller(kp=kp, 
                                                        desired_corner_locations = desired_corner_locations,
                                                        z_desired=z_desired,
                                                        focal_length_pixel=focal_length_pixel,
                                                        error_tolerance = e_tolerance, 
                                                        commands_topic = commands_topic, 
                                                        input_topic = input_topic,
                                                        ibvs_activate_topic = ibvs_activate_topic,
                                                        control_rate = control_rate, 
                                                        send_done_topic = send_done_topic,
                                                        robot_frame = robot_frame, 
                                                        camera_frame = camera_frame,
                                                        max_w = max_w, 
                                                        min_w = min_w, 
                                                        min_w_to_move = min_w_to_move,
                                                        max_v = max_v,
                                                        min_v = min_v,
                                                        min_v_to_move = min_v_to_move
                                                        )
    
    try:
        rospy.loginfo('The controller node is Running')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
