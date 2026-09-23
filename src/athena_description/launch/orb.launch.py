from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('athena_description'),
        'config',
        'zed2i_stereo_imu.yaml',
    )
    return LaunchDescription([
        Node(
                    package='orbslam3',
                    executable='stereo-inertial',   # check package.xml / setup.py for the real name
                    name='orb_slam3_stereo_inertial',
                    output='screen',
                    arguments=[
                        os.path.expanduser("~/thirdparty/ORB_SLAM3/Vocabulary/ORBvoc.txt"),
                        config_path,
                    ],
                    remappings=[
                        ('/camera/left', '/zed2i/left/image_rect_color'),
                        ('/camera/right', '/zed2i/right/image_rect_color'),
                        ('/imu', '/zed2i/imu/data'),
                    ],
                    parameters=[{'use_sim_time': True}],
                ),
    ])

# 1. get odom
# 2. Pose
# 3. 3D point cloud