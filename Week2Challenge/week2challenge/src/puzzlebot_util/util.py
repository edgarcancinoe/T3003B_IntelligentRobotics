import rospy

def get_global_params():
    # Reading parameters from the ROS parameter server
    params = {
        'freq': rospy.get_param('/node_rate', 100),
        'world_frame_name': rospy.get_param('/world_frame_name', 'odom'),
        'radius': rospy.get_param('/wheel_radius', 0.05),
        'track': rospy.get_param('/track_length', 0.19),
        'commands_topic': rospy.get_param('/commands_topic', 'cmd_vel'),
        'pose_topic': rospy.get_param('/pose_topic', 'pose'),
        'wl_topic': rospy.get_param('/wl_topic', 'wl'),
        'wr_topic': rospy.get_param('/wr_topic', 'wr')
    }

    return params
