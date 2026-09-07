from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share = get_package_share_directory("athena_description")
    urdf_file = os.path.join(package_share, "urdf", "athena_rover-6.urdf")

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
        
        Node (
            package="rtabmap_odom",
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': False,   
                'wait_imu_to_init': False, # temporary False
                'use_sim_time': True,
            }],
            remappings=[
                ('left/image_rect', '/zed2i/left/image_rect_color'),
                ('left/camera_info', '/zed2i/left/camera_info'),
                ('right/image_rect', '/zed2i/right/image_rect_color'),
                ('right/camera_info', '/zed2i/right/camera_info'),
                # ('imu', '/zed2i/imu/data'), temporary
            ]
        ),
        Node(
            package='athena_description',
            executable='Ackermann.py',
            name='ackermann_to_twist',
            output='screen',
            parameters=[{
                'ackermann_topic': '/ackermann_cmd',
                'cmd_vel_topic': '/cmd_vel',
                'stamped': True,
                'use_sim_time': True,
            }],
        ),
    ])
