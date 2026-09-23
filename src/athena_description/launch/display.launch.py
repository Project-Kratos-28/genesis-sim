from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('athena_description'),
        'urdf',
        'athena_rover-6.urdf',
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    steering_mode_arg = DeclareLaunchArgument(
        'steering_mode',
        default_value='ackermann',
        description=(
            "Initial wheel-kinematics strategy: 'ackermann' "
            "(front-steer-only, rear locked), 'differential' "
            "(skid-steer, no wheel turns), or 'independent' "
            "(full 4-wheel steer / crab / point turns). Can also be "
            "changed at runtime by publishing to /steering_mode."
        ),
    )


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
            output='screen',
            parameters=[{
                'steering_mode': LaunchConfiguration('steering_mode')
            }]
        ),
    ])
