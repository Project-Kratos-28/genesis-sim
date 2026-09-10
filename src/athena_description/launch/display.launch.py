from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():


    urdf_path = os.path.expanduser("~/Desktop/kratos/src/athena_description/urdf/athena_rover-6.urdf")

    with open(urdf_path, 'r') as f:
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
