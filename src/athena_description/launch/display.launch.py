from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_camera_launch_file = os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')

    zed_camera_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_camera_launch_file),
        launch_arguments={
            'camera_model': 'zed2i',
            'publish_tf': 'true', 
            'publish_map_tf': 'true',      
            'base_frame': 'base_footprint',  
            'cam_pose_frame': 'zed_camera_link',
        }.items()
    )

    urdf_file = os.path.expanduser("~/Desktop/kratos/src/athena_description/urdf/athena_rover-6.urdf")

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time':True,
            }]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='athena_description', 
            executable='genesis_launcher.py', 
            name='genesis_zed_bridge',
            output='screen'
        ),
        zed_camera_cmd        
    ])
