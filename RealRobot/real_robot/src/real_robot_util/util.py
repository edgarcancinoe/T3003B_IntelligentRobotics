import rospy
import numpy as np

def get_global_params():
    # Reading parameters from the ROS parameter server
    global_params = {
        'camera_frame': rospy.get_param('/camera_frame'),
        'object_frame': rospy.get_param('/object_frame'),
        'robot_frame': rospy.get_param('/robot_frame'),
        'inertial_frame': rospy.get_param('/inertial_frame'),
        'orientation_controller_topic': rospy.get_param('/orientation_controller_topic'),
        'pose_controller_topic': rospy.get_param('/pose_controller_topic'),
        'pose_control_activate_topic': rospy.get_param('/pose_control_activate_topic'),
        'aruco_detection_topic': rospy.get_param('/aruco_detection_topic'),
        'commands_topic': rospy.get_param('/commands_topic'),
        'lidar_topic': rospy.get_param('/lidar_topic'),
        'odometry_topic': rospy.get_param('/odometry_topic'),
        'ibvs_activate_topic': rospy.get_param('/ibvs_activate_topic'),
        'bug_activate_topic': rospy.get_param('/bug_activate_topic'),
        'wl_topic': rospy.get_param('/wl_topic'),
        'wr_topic': rospy.get_param('/wr_topic'),
        'control_rate': rospy.get_param('/control_rate'),
        'navigation_rate': rospy.get_param('/navigation_rate'),
        'starting_state': rospy.get_param('/starting_state'),
        'unlock': rospy.get_param('/unlock_topic'),
        'ibvs_done_topic': rospy.get_param('/ibvs_done_topic')
    }

    return global_params


def get_bug_navigation_params():
    bug_navigation_params = {
        'r_tolerance': rospy.get_param('/pose_controller/r_tolerance'),
        'horizontal_tolerance': rospy.get_param('/bug_navigation/horizontal_tolerance'),
        'frontal_tolerance': rospy.get_param('/bug_navigation/frontal_tolerance'),
        'lidar_resolution': rospy.get_param('/bug_navigation/lidar_resolution'),
        'v': rospy.get_param('/bug_navigation/v'),
        'w': rospy.get_param('/bug_navigation/w'),
        'min_d_hitpoint': rospy.get_param('/bug_navigation/bug2/min_d_hitpoint') # For Bug2
    }

    return {**bug_navigation_params, **get_global_params()}

def get_ibvs_params():
    ibvs_params = {
        'target_id': rospy.get_param('/target_id'),
        'target_aruco_size': rospy.get_param('/target_aruco_size'),
        'target_z_desired': rospy.get_param('/target_z_desired'),
        'target_corner_locations': np.array([rospy.get_param('/target_corner_locations')]),
        'station_id': rospy.get_param('/station_id'),
        'station_aruco_size': rospy.get_param('/station_aruco_size'),
        'station_z_desired': rospy.get_param('/station_z_desired'),
        'station_corner_locations': np.array([rospy.get_param('/station_corner_locations')]),
        'camera_matrix': np.array(rospy.get_param('/camera_matrix')),
        'distortion_coefficients': np.array(rospy.get_param('/distortion_coefficients')),
        'focal_length_pixel': rospy.get_param('/focal_length_pixel'),
        'max_v': rospy.get_param('/max_v'),
        'max_w': rospy.get_param('/max_w'),
        'min_v': rospy.get_param('/min_v'),
        'min_w': rospy.get_param('/min_w'),
        'min_v_to_move': rospy.get_param('/min_v_to_move'),
        'min_w_to_move': rospy.get_param('/min_w_to_move'),
        'kp': rospy.get_param('/ibvs/kp'),
        'e_tolerance': rospy.get_param('/ibvs/e_tolerance')
    }
    
    return {**ibvs_params, **get_global_params()}


def get_kalman_localisation_params():
    kalman_localisation_params = {
        'wheel_radius': rospy.get_param('/wheel_radius'),
        'track_length': rospy.get_param('/track_length'),
        'map_path': rospy.get_param('/map_path'),
        'k_l': rospy.get_param('/kalman_localisation/k_l'),
        'k_r': rospy.get_param('/kalman_localisation/k_r')
    }
    
    return {**kalman_localisation_params, **get_global_params()}


def get_map_localisation_params():
    map_localisation_params = {
        'map_path': rospy.get_param('/map_path'),
        'lidar_resolution': rospy.get_param('/map_localisation/lidar_resolution'),
        'lidar_offset': np.radians(rospy.get_param('/map_localisation/lidar_offset')), # Lidar offset is -90 for real lidar and 90 for simulated in gazebo.
        'num_ransac_iterations': rospy.get_param('/map_localisation/num_ransac_iterations'),
        'ransac_d_threshold': rospy.get_param('/map_localisation/ransac_d_threshold'),
        'min_ransac_inliers': rospy.get_param('/map_localisation/min_ransac_inliers'),
        'search_range': np.radians(rospy.get_param('/map_localisation/search_range')),
        'landmark_distance_threshold': rospy.get_param('/map_localisation/landmark_distance_threshold')
    }
    return {**map_localisation_params, **get_global_params()}
    

def get_orientation_controller_params():
    orientation_controller_params = {'kp': rospy.get_param('/orientation_controller/kp'), 
            'ki': rospy.get_param('/orientation_controller/ki'), 
            'e_tolerance': rospy.get_param('/orientation_controller/e_tolerance'),
            'min_w': rospy.get_param('/min_w'),
            'min_w_to_move': rospy.get_param('/min_w_to_move'),
            'max_w': rospy.get_param('/max_w'),
    }            
    return {**orientation_controller_params, **get_global_params()}


def get_pose_controller_params():
    pose_controller_params = {
        'k1': rospy.get_param('/pose_controller/k1'), 
        'k2': rospy.get_param('/pose_controller/k2'), 
        'beta': rospy.get_param('/pose_controller/beta'),
        'lambda': rospy.get_param('/pose_controller/lambda_'),
        'v_max': min(rospy.get_param('/pose_controller/v_max'), rospy.get_param('/max_v')),
        'r_tolerance': rospy.get_param('/pose_controller/r_tolerance'),
        'min_v': rospy.get_param('/min_v'),
    }
            
    return {**pose_controller_params, **get_global_params()}


def get_robot_state_pub_params():
    robot_state_params = {'update_rate': rospy.get_param('/rviz_publishing_rate')}
    return {**robot_state_params, **get_global_params()}


def get_uncertainty_localisation_params():
    uncertainty_localisation_params = {
        'wheel_radius': rospy.get_param('/wheel_radius'),
        'track_length': rospy.get_param('/track_length'),
        'k_l': rospy.get_param('/uncertainty_localisation/k_l'),
        'k_r': rospy.get_param('/uncertainty_localisation/k_r')
    }
    
    return {**uncertainty_localisation_params, **get_global_params()}


def get_vision_params():
    vision_params = {
        'target_id': rospy.get_param('/target_id'),
        'target_aruco_size': rospy.get_param('/target_aruco_size'),
        'target_z_desired': rospy.get_param('/target_z_desired'),
        'target_corner_locations': np.array([rospy.get_param('/target_corner_locations')]),
        'station_id': rospy.get_param('/station_id'),
        'station_aruco_size': rospy.get_param('/station_aruco_size'),
        'station_z_desired': rospy.get_param('/station_z_desired'),
        'station_corner_locations': np.array([rospy.get_param('/station_corner_locations')]),
        'camera_matrix': np.array(rospy.get_param('/camera_matrix')),
        'distortion_coeffs': np.array(rospy.get_param('/distortion_coefficients')),
        'target_detection_topic': rospy.get_param('/aruco_detection_topic'),
        'camera_topic': rospy.get_param('/camera_topic'),
    }

    return {**vision_params, **get_global_params()}