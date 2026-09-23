from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    common_params = {
        'frame_id': 'base_link',

        # Stereo input
        'subscribe_stereo': True,

        # IMU
        'wait_imu_to_init': True,

        # We want to see odometry information
        'subscribe_odom_info': True,

        # Start strict; timestamps should be fixed later if needed
        'approx_sync': True,

        'use_sim_time': True,
    }

    sync_params = {
        'approx_sync': False,
        'use_sim_time': True,
    }

    return LaunchDescription([
        Node(
            package='rtabmap_sync',
            executable='stereo_sync',
            name='stereo_sync',
            output='screen',

            parameters=[sync_params],

            remappings=[
                ('left/image_rect',
                 '/zed2i/left/image_rect_color'),

                ('left/camera_info',
                 '/zed2i/left/camera_info'),

                ('right/image_rect',
                 '/zed2i/right/image_rect_color'),

                ('right/camera_info',
                 '/zed2i/right/camera_info'),
            ],
        ),

        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',

            parameters=[common_params],

            remappings=[
                ('imu', '/zed2i/imu/data'),

                ('odom', '/rtabmap/odom'),
            ],
        ),

    ])