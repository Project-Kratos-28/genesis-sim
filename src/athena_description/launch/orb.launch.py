from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
                    package='orbslam3',
                    executable='stereo-inertial',   # check package.xml / setup.py for the real name
                    name='orb_slam3_stereo_inertial',
                    output='screen',
                    arguments=[
                        os.path.expanduser("~/Desktop/kratos/src/athena_description/config/ORBvoc.txt"),
                        os.path.expanduser("~/Desktop/kratos/src/athena_description/config/zed2i_stereo_imu.yaml"),
                    ],
                    remappings=[
                        ('/camera/left', '/zed2i/left/image_rect_color'),
                        ('/camera/right', '/zed2i/right/image_rect_color'),
                        ('/imu', '/zed2i/imu/data'),
                    ],
                    parameters=[{'use_sim_time': True}],
                ),
    ])