"""Stereo + IMU visual odometry (VIO) on the simulated ZED2i.

Starts the regular simulation (`display.launch.py`: robot_state_publisher,
joint_state_publisher_gui and the Genesis bridge) and adds RTAB-Map's
`stereo_odometry` node, consuming the ZED-compatible topics published by the
bridge:

    /zed/zed_node/left/image_rect_color
    /zed/zed_node/right/image_rect_color
    /zed/zed_node/left/camera_info
    /zed/zed_node/right/camera_info
    /zed/zed_node/imu/data

Because those are the names the real ZED ROS 2 wrapper uses, the same node
configuration runs unchanged against the physical camera (just drop the
simulation launch). The one extra node is `imu_filter_madgwick`, which turns the
bridge's raw `imu/data_raw` into `imu/data` with orientation -- the same
arrangement ZED2i + RTAB-Map setups use on hardware. The only sim-specific
choices are `use_sim_time` and `publish_tf:=false`: TF stays owned by the bridge
(`map -> base_footprint`) so the estimated trajectory cannot fight the
ground-truth tree. Read the estimate from `/odom` (or `/rtabmap/odom_info`).

Note: the Martian terrain mesh is untextured, so feature tracking relies on the
shading facets and the rover's own shadow. Expect noisier odometry than on the
real rover, where the ground has texture.

The Genesis viewer can be shown alongside `rtabmap_viz`, but be aware that the
viewer and the simulated ZED cameras share the GPU: with the window open the
camera reads are ~25x slower, so simulated time crawls (set `show_viewer:=false`
when you care about VIO accuracy or run a long validation drive).

Usage:
    ros2 launch athena_description rtabmap_vio.launch.py
    ros2 launch athena_description rtabmap_vio.launch.py show_viewer:=false
    ros2 launch athena_description rtabmap_vio.launch.py steering_mode:=independent
    # accuracy check against the simulator's ground-truth TF:
    ros2 launch athena_description rtabmap_vio.launch.py ground_truth_frame_id:=map \\
        show_viewer:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ZED = "/zed/zed_node"
LEFT_FRAME = "zed_left_camera_optical_frame"


def generate_launch_description():
    pkg = get_package_share_directory("athena_description")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "steering_mode",
            default_value="ackermann",
            description="Wheel-kinematics strategy used by the Genesis bridge.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Drive the RTAB-Map nodes from /clock (must stay true).",
        ),
        # The Genesis viewer and the simulated ZED cameras share the GPU: with
        # the window open camera reads are ~25x slower and simulated time
        # crawls. The viewer is therefore optional -- leave it on to watch the
        # rover in Genesis, pass `show_viewer:=false` for accuracy runs.
        DeclareLaunchArgument(
            "show_viewer",
            default_value="true",
            description=(
                "Open the Genesis viewer window next to rtabmap_viz. Set to "
                "'false' for sensor/VIO accuracy work (viewer starves camera reads)."
            ),
        ),
        # Optional validation: point RTAB-Map at the bridge's ground-truth TF
        # (`map -> base_footprint`). RTAB-Map then aligns the estimate and the
        # truth itself and logs the per-frame error, which avoids having to
        # guess the odom frame's convention outside the node. Leave empty for
        # normal operation (and on hardware, where there is no ground truth).
        DeclareLaunchArgument(
            "ground_truth_frame_id",
            default_value="",
            description="Ground-truth frame for validation, e.g. 'map'. Empty disables it.",
        ),
        DeclareLaunchArgument(
            "ground_truth_base_frame_id",
            default_value="base_footprint",
            description="Base frame paired with ground_truth_frame_id.",
        ),

        # Simulation: robot_state_publisher + joint_state_publisher_gui + bridge.
        # The viewer is forwarded from `show_viewer` (default on).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "display.launch.py")),
            launch_arguments={
                "steering_mode": LaunchConfiguration("steering_mode"),
                "show_viewer": LaunchConfiguration("show_viewer"),
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=["--frame-id", "map", "--child-frame-id", "odom"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # Orientation for the IMU, exactly as on the real ZED2i: the driver
        # publishes raw `imu/data_raw`, and a Madgwick filter publishes the
        # `imu/data` consumed by VIO. Doing this in the loop (instead of
        # forwarding ground-truth orientation) keeps the estimator honest and
        # mirrors the hardware setup.
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            output="screen",
            parameters=[{
                "use_mag": False,
                "world_frame": "enu",
                "publish_tf": False,
                "fixed_frame": "zed_imu_link",
                "use_sim_time": use_sim_time,
            }],
            remappings=[
                ("imu/data_raw", f"{ZED}/imu/data_raw"),
                ("imu/data", f"{ZED}/imu/data"),
            ],
        ),

        # Stereo + IMU visual odometry
        Node(
            package="rtabmap_odom",
            executable="stereo_odometry",
            name="stereo_odometry",
            output="screen",
            parameters=[{
                "frame_id": LEFT_FRAME,
                "odom_frame_id": "odom",
                "publish_tf": False,
                "wait_imu_to_init": True,
                "always_check_imu_tf": True,
                # approx_sync: image at 10 Hz, IMU at 100 Hz -- allow up to 150 ms
                # slop so a single slow camera render does not break synchronisation.
                "approx_sync": True,
                "approx_sync_max_interval": 0.15,
                "qos": 1,           # reliable: the bridge publishes with default QoS
                "qos_camera_info": 1,
                "qos_imu": 1,
                "subscribe_stereo": True,
                "use_sim_time": use_sim_time,
                "ground_truth_frame_id": LaunchConfiguration("ground_truth_frame_id"),
                "ground_truth_base_frame_id": LaunchConfiguration("ground_truth_base_frame_id"),
                # VIO tuning: frame-to-map, ORB/GFTT features.
                # RTAB-Map declares its own parameters as strings, so these must
                # be quoted too.
                "Odom/Strategy": "0",           # frame-to-map
                "OdomF2M/MaxSize": "2000",
                # GFTT (Shi-Tomasi) corners -- robust on low-texture terrain shading.
                "Vis/FeatureType": "6",
                "Vis/MaxFeatures": "1000",
                # Lowered from 15: the simulated terrain has weak texture and
                # ~11-19 matches per frame is typical.  10 inliers is enough to
                # constrain 6-DoF pose well.
                "Vis/MinInliers": "10",
                # Wider search window so matches survive large inter-frame motion.
                "Vis/CorGuessWinSize": "50",
                # Denser feature grid so the rover's own body provides enough
                # corners even when the terrain is flat / featureless.
                "GFTT/MinDistance": "2.0",
                # Optical-flow pyramid levels: 3 -> 4 helps track features across
                # the larger apparent motion caused by the camera bouncing over rocks.
                "OdomF2M/BundleAdjustment": "0",
            }],
            remappings=[
                ("left/image_rect", f"{ZED}/left/image_rect_color"),
                ("right/image_rect", f"{ZED}/right/image_rect_color"),
                ("left/camera_info", f"{ZED}/left/camera_info"),
                ("right/camera_info", f"{ZED}/right/camera_info"),
                ("imu", f"{ZED}/imu/data"),
            ],
        ),

        # Live view of features / odometry
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            name="rtabmap_viz",
            output="screen",
            parameters=[{
                "frame_id": LEFT_FRAME,
                # "odom_frame_id": "odom",
                "subscribe_stereo": True,
                "approx_sync": True,
                "approx_sync_max_interval": 0.15,
                "use_sim_time": use_sim_time,
            }],
            remappings=[
                ("left/image_rect", f"{ZED}/left/image_rect_color"),
                ("right/image_rect", f"{ZED}/right/image_rect_color"),
                ("left/camera_info", f"{ZED}/left/camera_info"),
                ("right/camera_info", f"{ZED}/right/camera_info"),
                ("imu", f"{ZED}/imu/data"),
                ("odom", "/odom"),
                ("odom_info", "/odom_info")
            ],
        ),
    ])
