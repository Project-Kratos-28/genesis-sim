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
 
LINKS_TO_KEEP = [
    "base_footprint",
    "zed2i_camera_center",
    "zed2i_left_camera_frame_optical",
    "zed2i_right_camera_frame_optical",
]

## Joint Controller gains
STEER_KP = 800.0
STEER_KV = 40.0
WHEEL_KP = 0.0
WHEEL_KV = 30.0


## Camera
WIDTH, HEIGHT, FOV_DEG = 1280, 720, 110.0
BASELINE = 0.12
CAMERA_DECIMATION = 1
POINTCLOUD_STRIDE = 4
DEPTH_MIN, DEPTH_MAX = 0.05, 30.0

## Frames
MAP_FRAME = "map"
BASE_FOOTPRINT_FRAME = "base_footprint"
ODOM_CHILD_FRAME = "base_link"
IMU_FRAME = "zed2i_camera_center"
LEFT_CAM_FRAME = "zed2i_left_camera_frame_optical"
RIGHT_CAM_FRAME = "zed2i_right_camera_frame_optical"


## Topics 
TOPIC_LEFT_IMAGE = "/left/image_rect"
TOPIC_RIGHT_IMAGE = "/right/image_rect"
TOPIC_LEFT_INFO = "/left/camera_info"
TOPIC_RIGHT_INFO = "/right/camera_info"
TOPIC_IMU_RAW = "/imu/data_raw"
TOPIC_IMU = "/imu/data"
TOPIC_CLOCK = "/clock"
TOPIC_ODOM = "/odom"
TOPIC_LEFT_PC = "/point_cloud/left"
TOPIC_RIGHT_PC = "/point_cloud/right"
TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_STEERING_MODE = "/steering_mode"


## Misc 
DEFAULT_STEERING_MODE = "ackermann"
DEBUG_PRINT_EVERY = 100