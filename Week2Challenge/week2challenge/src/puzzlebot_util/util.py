import rospy

def get_global_params():
    # Reading parameters from the ROS parameter server
    params = {
        'freq': rospy.get_param('/node_rate', 100),
        'inertial_frame_name': rospy.get_param('/inertial_frame_name', 'odom'),
        'robot_frame_name': rospy.get_param('/robot_frame_name', 'base_link'),
        'wheel_radius': rospy.get_param('/wheel_radius', 0.05),
        'track_length': rospy.get_param('/track_length', 0.19),
        'commands_topic': rospy.get_param('/commands_topic', 'cmd_vel'),
        'pose_topic': rospy.get_param('/pose_topic', 'pose'),
        'wl_topic': rospy.get_param('/wl_topic', 'wl'),
        'wr_topic': rospy.get_param('/wr_topic', 'wr'),
        'odometry_topic': rospy.get_param('/odometry_topic', 'odom'),
        'starting_state': rospy.get_param('/starting_state', {'x': 0.0, 'y': 0.0, 'theta': 0.0})
    }

    return params
