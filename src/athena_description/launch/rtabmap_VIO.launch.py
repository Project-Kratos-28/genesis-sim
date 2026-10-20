from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    common_params = {
        'frame_id': 'base_link',
        'subscribe_stereo': True,
        'wait_imu_to_init': True,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'approx_sync_max_interval': 0.02,
        'use_sim_time': True,
        'Odom/ResetCountdown': '1',
        'Vis/MinInliers': '15',
        'GFTT/QualityLevel': '0.0005',
    }

    sync_params = {
        'approx_sync': True,
        'approx_sync_max_interval': 0.02,
        'use_sim_time': True,
    }

    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
                'gain': 0.1,
                'use_sim_time': True,
            }],
            remappings=[
                ('imu/data_raw', 'imu/data_raw'),
                ('imu/data', 'imu/data'),
            ],

        ),

        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',

            parameters=[common_params],

            remappings=[
                ('imu', 'imu/data'),

                ('odom', '/rtabmap/odom'),
            ],
        ),
        Node(
            package='rtabmap_sync',
            executable='stereo_sync',
            name='stereo_sync',
            output='screen',
            parameters=[sync_params],
        ),

    ])