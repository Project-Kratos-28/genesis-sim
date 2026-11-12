import math
import os


## Assets
URDF_PATH = os.path.expanduser(
    "~/Desktop/kratos/src/athena_description/urdf/athena_rover-6.urdf"
)

TERRAIN_PATH = os.path.expanduser(
    "~/Desktop/kratos/src/athena_description/urdf/payloads/mars_terrain_lowres.stl"
)

TERRAIN_POS = (10.0, 4.0, 0.0)
TERRAIN_FRICTION = 0.8


## Simulation 
SIM_DT = 0.01
SHOW_VIEWER = True
 
WHEEL_BOTTOM_TO_FOOTPRINT = -0.2145
SPAWN_CLEARANCE = 1.5
SPAWN_Z = -WHEEL_BOTTOM_TO_FOOTPRINT + SPAWN_CLEARANCE

# Steps simulated with zero wheel command before the ROS clock starts. The rover
# is spawned SPAWN_CLEARANCE above the terrain, and the free fall that follows
# reads as ~0 g on the IMU: Madgwick cannot determine gravity, so it latches a
# wrong attitude and the VIO starts with a bogus initial pose. Settling first
# means every published sensor sample is post-landing.
SETTLE_STEPS = 300
 
LINKS_TO_KEEP = [
    "base_footprint",
]

## Visual landmarks
#
# The terrain STL carries no UVs, so it cannot be textured, and a uniform white
# mesh gives a stereo/VIO stack nothing to track (features come only from facet
# shading, and the rover's own shadow -- which is static in the image -- pulls
# the estimate towards zero). Scatter static, non-colliding boulders so the
# simulated cameras see the kind of visual structure the real terrain has.
VISUAL_LANDMARKS = True
LANDMARK_COUNT = 160
LANDMARK_SEED = 11
LANDMARK_RADIUS = (0.10, 0.40)  # metres, boulder radius range
LANDMARK_EXCLUSION = 2.5  # metres around the spawn kept clear

## Joint Controller gains
STEER_KP = 800.0
STEER_KV = 40.0
WHEEL_KP = 0.0
WHEEL_KV = 30.0


## Simulated ZED2i (stereo camera pair + IMU)
#
# The rig mirrors the URDF (`zed_left/right_camera_frame`, `zed_imu_link`).
# Both render cameras sit exactly on those frames' origins, which lie *inside*
# the camera housing mesh (the ZED2i's optical centre is 25 mm behind the lens
# face). The housing is therefore clipped rather than the camera being pushed
# forward: ZED_NEAR (3 cm) is just beyond the 2.5 cm of body/lens geometry in
# front of the optical centre, so images stay clean while the published frames,
# baseline and camera-IMU lever arm all match the URDF exactly.
ZED_ATTACH_LINK = "base_footprint"  # only link kept by merge_fixed_links

ZED_WIDTH = 1280  # ZED2i HD720
ZED_HEIGHT = 720
# Vertical FOV chosen so the render intrinsics match the ZED2i HD720 nominal
# calibration: fy = (H/2) / tan(vfov/2) = 527.9 px (fx == fy, square pixels).
ZED_VERTICAL_FOV = 68.59  # degrees

ZED_FX = (ZED_HEIGHT / 2.0) / math.tan(math.radians(ZED_VERTICAL_FOV) / 2.0)
ZED_FY = ZED_FX
ZED_CX = ZED_WIDTH / 2.0
ZED_CY = ZED_HEIGHT / 2.0
ZED_BASELINE = 0.12  # metres, matches the URDF (y = +/- 0.06)
ZED_NEAR = 0.03  # metres; clips the camera's own housing

# Render camera pose in `ZED_ATTACH_LINK`; base_footprint -> base_link is z +0.155.
# These are the URDF optical-frame origins (base_link x -0.01, z +0.110).
ZED_CAMERA_X = 0.245
ZED_CAMERA_Y = 0.06  # left camera; the right one is mirrored in Y
ZED_CAMERA_Z = 0.265
ZED_IMU_POS = (0.255, 0.0, 0.265)

# Sensor output rates, in simulated (not wall-clock) time.
ZED_IMAGE_RATE = 10.0  # Hz
ZED_IMU_RATE = 100.0  # Hz, one sample per physics step

# IMU noise model, order-of-magnitude BMI088 figures (the ZED2i's IMU).
ZED_IMU_ACC_NOISE = 0.0016  # m/s^2, per sample
ZED_IMU_GYRO_NOISE = 0.00011  # rad/s, per sample
ZED_IMU_ACC_BIAS = (0.02, -0.015, 0.01)  # m/s^2
ZED_IMU_GYRO_BIAS = (0.001, -0.0008, 0.0006)  # rad/s


## Simulated Livox MID-360 LiDAR
#
# 360 deg x (-7 .. +52 deg) coverage, matching the MID-360 datasheet. The
# simulated scan is a uniform spherical grid (the real unit uses a
# non-repetitive rosette pattern), so point density is illustrative rather
# than bit-identical to the hardware.
#
# Attached to the link that survives merge_fixed_links, at the URDF joint
# base_link -> livox_mid360_link (0, 0, 0.175). base_footprint -> base_link is
# z +0.155, so the sensor origin is 0.330 m above base_footprint and its axes
# are identical to base_footprint (every joint in the chain has rpy 0).
LIVOX_ATTACH_LINK = "base_footprint"
LIVOX_POS = (0.0, 0.0, 0.330)

LIVOX_VERTICAL_FOV = (-7.0, 52.0)  # degrees, (min, max) elevation
LIVOX_BEAMS_H = 2000  # azimuth samples per scan (uniform, 360 deg)
LIVOX_BEAMS_V = 32  # elevation rings per scan

LIVOX_MIN_RANGE = 0.1  # metres, MID-360 blind zone
LIVOX_MAX_RANGE = 100.0  # metres
LIVOX_RANGE_NOISE = 0.01  # metres, 1-sigma range noise per point

LIVOX_SCAN_RATE = 10.0  # Hz, in simulated time
LIVOX_FRAME = "livox_mid360_link"


## Frames
MAP_FRAME = "map"
BASE_FOOTPRINT_FRAME = "base_footprint"
ODOM_CHILD_FRAME = "base_link"
ZED_LEFT_FRAME = "zed_left_camera_optical_frame"
ZED_RIGHT_FRAME = "zed_right_camera_optical_frame"
ZED_IMU_FRAME = "zed_imu_link"


## Topics 
TOPIC_CLOCK = "/clock"
TOPIC_ODOM = "/odom"
TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_STEERING_MODE = "/steering_mode"

# ZED ROS 2 wrapper-compatible names, so a VIO stack tuned here runs unchanged
# against the real camera.
TOPIC_ZED_LEFT_IMAGE = "/zed/zed_node/left/image_rect_color"
TOPIC_ZED_RIGHT_IMAGE = "/zed/zed_node/right/image_rect_color"
TOPIC_ZED_LEFT_INFO = "/zed/zed_node/left/camera_info"
TOPIC_ZED_RIGHT_INFO = "/zed/zed_node/right/camera_info"
TOPIC_ZED_IMU_RAW = "/zed/zed_node/imu/data_raw"

# Livox ROS 2 driver-compatible name. livox_ros_driver2 publishes
# sensor_msgs/PointCloud2 here when configured with xfer_format:=1 (default is
# the custom livox_ros_driver2/msg/CustomMsg on the same topic name).
TOPIC_LIVOX_POINTS = "/livox/lidar"


## Misc 
DEFAULT_STEERING_MODE = "ackermann"
DEBUG_PRINT_EVERY = 100

# Path where the bridge persists its last sim_time so a restart can continue
# the clock monotonically without waiting for /clock subscription callbacks
# (which are unreliable across process restarts due to VOLATILE QoS).
CLOCK_PERSIST_PATH = os.path.expanduser("~/.ros/genesis_bridge_clock.txt")

# Exclusive lock held for the lifetime of a bridge process. Two bridges would
# each publish /clock from a different persisted offset, making time jump
# backwards and clearing every downstream TF buffer -- refuse to start.
CLOCK_LOCK_PATH = os.path.expanduser("~/.ros/genesis_bridge.lock")
