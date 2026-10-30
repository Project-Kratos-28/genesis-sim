# AGENTS.md — Kratos Workspace Guide

Welcome to the **Kratos** repository. This document serves as the authoritative operational manual and architectural guide for AI agents and developers working in this workspace.

---

## 1. Project Overview

**Kratos** is a ROS 2 Jazzy robotics workspace hosting the **Athena Rover** (`athena_rover`), an autonomous 4-wheeled planetary rover designed for rough Martian terrain exploration. The workspace integrates high-performance GPU physics simulation with modern Visual-Inertial Odometry (VIO) and multi-mode rover kinematics.

### Key Capabilities & Technologies
- **Robotic Platform**: Athena Rover — 4-wheel drive, 4-wheel independent steering rover with dual bogey suspension.
- **Physics Simulation**: [Genesis](https://genesis-world.readthedocs.io/) GPU-accelerated physical simulation (`gs.init(backend=gs.gpu)`), running custom Martian elevation terrain.
- **Perception & Sensors**: Simulated Stereolabs ZED2i stereo camera (1280x720, 110° FOV, 0.12 m baseline) and 6-DOF IMU.
- **Visual-Inertial Odometry (VIO)**: [RTAB-Map](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/RTAB-Map) (`stereo_odometry` + `stereo_sync`) fused with filtered IMU orientation via `imu_filter_madgwick`.
- **Kinematics Strategies**: Modular runtime-switchable steering controllers:
  - **Ackermann Steering** (front-wheel steering, rear locked)
  - **Differential Drive** (skid-steer mode, 0° steer angle)
  - **Independent 4-Wheel Steering** (all-wheel steer, crab walking, point turns)
- **Target OS / ROS Distribution**: Ubuntu 24.04 LTS with ROS 2 Jazzy Jalisco.

---

## 2. System Architecture & Data Flow

```
                      +---------------------------------------+
                      |       Genesis Physics Engine          |
                      |  - Martian Terrain Mesh / Heightfield |
                      |  - Athena Rover URDF Dynamics         |
                      |  - GPU Raytracing Cameras & IMU       |
                      +-------------------+-------------------+
                                          |
                                          v
                      +---------------------------------------+
                      |    genesis_bridge (SimulationRunner)  |
                      |  - bridge_node: /cmd_vel & /steering  |
                      |  - rover_controller: PD Joint gains   |
                      |  - camera_model & publishers          |
                      +-------------------+-------------------+
                                          |
         +--------------------------------+--------------------------------+
         |                                |                                |
         v                                v                                v
   /clock (sim_time)         /left/image_rect, /right/image_rect        /imu/data_raw
   TF: map -> base_footprint /left/camera_info, /right/camera_info      (accel + gyro)
                                          |                                |
                                          |                                v
                                          |                    +-----------------------+
                                          |                    |  imu_filter_madgwick  |
                                          |                    |  (computes quat pose) |
                                          |                    +-----------+-----------+
                                          |                                |
                                          |                                v
                                          |                           /imu/data
                                          |                       (accel + gyro + quat)
                                          |                                |
                                          +---------------+----------------+
                                                          |
                                                          v
                                          +-------------------------------+
                                          |    rtabmap_odom & sync        |
                                          |   - stereo_sync               |
                                          |   - stereo_odometry           |
                                          +---------------+---------------+
                                                          |
                                                          v
                                                    /rtabmap/odom
                                                  /rtabmap/odom_info
```

---

## 3. Directory Structure

```
kratos/
├── AGENTS.md                                   # This agent and developer operational guide
├── kratos                                      # Shell helper: activates python venv & ROS 2 Jazzy
├── .agents/
│   └── skills/
│       └── kratos-rover-vio/
│           └── SKILLS.md                       # Task-specific agent skills and invariant rules
├── .gitignore                                  # Git exclusion rules
├── .gitmodules                                 # Submodule mappings (vision_opencv, zed-ros2-wrapper)
│
├── src/
│   ├── athena_description/                     # Primary robot package (URDF, sim bridge, VIO launch)
│   │   ├── CMakeLists.txt                      # Ament CMake build configuration
│   │   ├── package.xml                         # ROS 2 package dependencies
│   │   │
│   │   ├── config/
│   │   │   └── zed2i_stereo_imu.yaml           # Historical camera/IMU calibration parameters
│   │   │
│   │   ├── launch/
│   │   │   ├── display.launch.py               # Main launcher: robot_state_publisher + genesis_launcher + check.py
│   │   │   └── rtabmap_VIO.launch.py           # VIO launcher: imu_filter_madgwick + stereo_odometry + stereo_sync
│   │   │
│   │   ├── meshes/                             # STL 3D models for visualization & collision
│   │   │   ├── visual/                         # Visual meshes: chassis, bogey_arm, wheel, stearing, etc.
│   │   │   └── collision/                      # Collision meshes for physics engine
│   │   │
│   │   ├── rviz/
│   │   │   └── rtabmap_vio.rviz                # Preconfigured RViz display for VIO inspection
│   │   │
│   │   ├── script/
│   │   │   ├── genesis_launcher.py             # Thin executable entrypoint launching genesis_bridge.runner
│   │   │   ├── check.py                        # Diagnostic node verifying IMU gravity vector against TF
│   │   │   ├── Ackermann.py                    # Standalone Ackermann kinematics prototype
│   │   │   │
│   │   │   ├── genesis_bridge/                 # Modular Genesis-to-ROS 2 simulation bridge
│   │   │   │   ├── __init__.py
│   │   │   │   ├── config.py                   # Central simulation constants, topics, gains, frames
│   │   │   │   ├── runner.py                   # SimulationRunner: main physics & render stepping loop
│   │   │   │   ├── bridge_node.py              # GenesisZedBridge ROS 2 Node (cmd_vel & steering subscribers)
│   │   │   │   ├── scene.py                    # Genesis scene setup (terrain, rover URDF, cameras, IMU)
│   │   │   │   ├── rover_controller.py         # Genesis DOF PD gain config and actuator commands
│   │   │   │   ├── camera_model.py             # Camera intrinsics, projection matrix, FOV math
│   │   │   │   ├── pointcloud.py               # Depth image to PointCloud2 generator
│   │   │   │   └── publishers.py               # ROS 2 publishers (images, camera_info, IMU, clock, TF)
│   │   │   │
│   │   │   └── steering/                       # Modular wheel kinematics engine
│   │   │       ├── __init__.py                 # Factory function make_steering_mode(name)
│   │   │       ├── base.py                     # Abstract SteeringMode interface
│   │   │       ├── wheel_config.py             # Geometric constants (wheelbase, track, joint names)
│   │   │       ├── ackermann.py                # Ackermann front-wheel steering implementation
│   │   │       ├── differential.py             # Differential / skid-steer kinematics implementation
│   │   │       └── independent.py              # 4-wheel independent steering implementation
│   │   │
│   │   └── urdf/
│   │       ├── athena_rover-6.urdf             # Plain URDF loaded directly by Genesis simulation
│   │       ├── athena_rover-6.urdf.xacro       # Parameterized Xacro model (with Gazebo plugin hooks)
│   │       └── payloads/                       # Simulation terrain assets
│   │           ├── mars_terrain_lowres.stl     # Active Martian terrain STL loaded in scene.py
│   │           ├── martian_terrain.stl         # High-resolution Martian terrain
│   │           ├── martian_terrain_heightfield.npy # NumPy elevation heightfield
│   │           └── model.glb / model.stl       # Blender/FBX source models
│   │
│   ├── vision_opencv/                          # [Submodule] cv_bridge & image_geometry for ROS 2 Jazzy
│   └── zed-ros2-wrapper/                       # [Submodule] Official Stereolabs ZED ROS 2 integration
│
├── build/                                      # colcon build artifacts (ignored by git)
├── install/                                    # colcon installation tree (ignored by git)
└── log/                                        # colcon build & test logs (ignored by git)
```

---

## 4. Environment & Prerequisites

1. **Host Environment**: Ubuntu 24.04 (Linux x86_64), NVIDIA GPU with CUDA drivers installed.
2. **ROS 2 Distribution**: ROS 2 Jazzy Jalisco (`/opt/ros/jazzy/setup.bash`).
3. **Python Virtual Environment**: `~/genesis_env` containing Genesis (`genesis-world`), PyTorch with CUDA, and SciPy.
4. **Required ROS 2 Packages**:
   - `rtabmap_odom`, `rtabmap_sync`, `rtabmap_util`, `rtabmap_msgs`
   - `imu_filter_madgwick`
   - `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `xacro`

---

## 5. Build, Sourcing & Running Guide

### 5.1 Environment Sourcing
Use the root convenience script or run manually:
```bash
# Quick source:
source kratos

# Or manually:
source ~/genesis_env/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 5.2 Building the Workspace
Always build with `--symlink-install` so Python script edits do not require rebuilding:
```bash
colcon build --symlink-install --packages-select athena_description
source install/setup.bash
```

### 5.3 Running the Simulation
Launch the Genesis simulation, robot state publisher, and bridge:
```bash
ros2 launch athena_description display.launch.py
```
*Optional argument*: specify the initial steering mode:
```bash
ros2 launch athena_description display.launch.py steering_mode:=ackermann
# Or: steering_mode:=differential, steering_mode:=independent
```

### 5.4 Running Visual-Inertial Odometry (RTAB-Map)
In a separate terminal (with `source kratos`):
```bash
ros2 launch athena_description rtabmap_VIO.launch.py
```

### 5.5 Visualizing in RViz
```bash
rviz2 -d src/athena_description/rviz/rtabmap_vio.rviz
```

### 5.6 Teleoperating the Rover
Publish velocity commands to `/cmd_vel`:
```bash
# Drive forward at 0.5 m/s with 0.2 rad/s turn
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}" -r 10
```

Switch steering mode dynamically at runtime:
```bash
ros2 topic pub /steering_mode std_msgs/msg/String "{data: 'independent'}" --once
```

---

## 6. Kinematics & Rover Geometry

All rover geometry values are centralized in `src/athena_description/script/steering/wheel_config.py`.

### Rover Specifications
- **Wheel Radius**: $R = 0.115\text{ m}$
- **Front Axle X**: $+0.39291\text{ m}$, **Rear Axle X**: $-0.34458\text{ m}$
- **Wheelbase ($L$)**: $0.73749\text{ m}$
- **Front Track ($W_f$)**: $0.78362\text{ m}$ (offsets $y = \pm 0.39181\text{ m}$)
- **Rear Track ($W_r$)**: $0.75796\text{ m}$ (offsets $y = \pm 0.37898\text{ m}$)
- **Max Steer Angle**: $\pm 1.57\text{ rad}$ ($\pm 90^\circ$)

### Joint Mappings
| Wheel Position | Steer Joint Name | Wheel Spin Joint Name | Position $(x, y)$ |
| :--- | :--- | :--- | :--- |
| **Front Left** | `steer_front_left` | `wheel_front_left_spin` | $(+0.39291, +0.39181)$ |
| **Front Right** | `steer_front_right` | `wheel_front_right_spin` | $(+0.39291, -0.39181)$ |
| **Rear Left** | `steer_rear_left` | `wheel_rear_left_spin` | $(-0.34458, +0.37898)$ |
| **Rear Right** | `steer_rear_right` | `wheel_rear_right_spin` | $(-0.34458, -0.37898)$ |

### Steering Modes
1. **`ackermann`**: Front wheels compute Ackermann angles based on wheelbase and instantaneous center of curvature (ICC); rear wheels remain locked at $0\text{ rad}$.
2. **`differential`**: All 4 steer joints locked at $0\text{ rad}$. Left and right wheels drive at differential speeds ($v_w = (v \mp \omega_z \cdot y) / R$) for skid-steering.
3. **`independent`**: All 4 wheels steer independently. Supports crab steering, diagonal translation, and zero-radius point turns.

---

## 7. ROS Interface & Topics Contract

### Published by `genesis_bridge`
| Topic | Type | Rate | Description |
| :--- | :--- | :--- | :--- |
| `/clock` | `rosgraph_msgs/msg/Clock` | 100 Hz | Simulation wall-clock time from Genesis |
| `/left/image_rect` | `sensor_msgs/msg/Image` | 20–100 Hz | Left rectified camera frame (`rgb8`) |
| `/right/image_rect` | `sensor_msgs/msg/Image` | 20–100 Hz | Right rectified camera frame (`rgb8`) |
| `/left/camera_info` | `sensor_msgs/msg/CameraInfo` | 20–100 Hz | Left camera intrinsics & projection $P$ |
| `/right/camera_info` | `sensor_msgs/msg/CameraInfo` | 20–100 Hz | Right camera intrinsics with baseline offset $-f \cdot b$ |
| `/point_cloud/left` | `sensor_msgs/msg/PointCloud2` | 20–100 Hz | Left depth converted to point cloud |
| `/point_cloud/right` | `sensor_msgs/msg/PointCloud2` | 20–100 Hz | Right depth converted to point cloud |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 100 Hz | Raw linear acceleration and angular velocity (no orientation) |

### Processed / Published by VIO Nodes
| Topic | Type | Publisher Node | Description |
| :--- | :--- | :--- | :--- |
| `/imu/data` | `sensor_msgs/msg/Imu` | `imu_filter_madgwick` | Filtered IMU with estimated orientation quaternion |
| `/rtabmap/odom` | `nav_msgs/msg/Odometry` | `rtabmap_odom` | Visual-inertial estimated odometry |
| `/rtabmap/odom_info` | `rtabmap_msgs/msg/OdomInfo` | `rtabmap_odom` | Diagnostic odometry metrics (inliers, quality, loss) |

### Subscribed by `genesis_bridge`
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Linear velocity $x$ and angular velocity $z$ commands |
| `/steering_mode` | `std_msgs/msg/String` | Dynamically switch to `"ackermann"`, `"differential"`, or `"independent"` |

### TF Frame Hierarchy
```
map
 └── base_footprint
      └── base_link
           ├── bogey_left / bogey_right
           │    └── steer_coupler_*
           │         └── wheel_*
           └── zed2i_camera_link
                └── zed2i_camera_center (IMU Frame)
                     ├── zed2i_left_camera_frame_optical
                     └── zed2i_right_camera_frame_optical
```

---

## 8. Invariants & Hard Rules for Agents

When contributing to this repository, AI agents **MUST** strictly adhere to the following decisions:

1. **NO ORB-SLAM3**:
   - ORB-SLAM3 has been completely deprecated and removed from this project.
   - **NEVER** re-add ORB-SLAM3, Pangolin, `orb_slam3_ros`, or `orb.launch.py`.
   - VIO is exclusively implemented using RTAB-Map (`rtabmap_odom::stereo_odometry`).
2. **Keep the Bridge Modular**:
   - Do **NOT** revert `genesis_launcher.py` into a monolithic script.
   - All simulation lifecycle belongs in `src/athena_description/script/genesis_bridge/`.
   - All steering logic belongs in `src/athena_description/script/steering/`.
3. **Simulation Time Contract**:
   - Genesis drives simulated time. `/clock` is published by the bridge.
   - Every downstream ROS 2 node (RTAB-Map, Madgwick, robot_state_publisher) **MUST** have parameter `use_sim_time: True`.
4. **IMU Orientation Requirement**:
   - Genesis simulator IMU provides raw acceleration and gyro, **not** valid orientation.
   - RTAB-Map's `stereo_odometry` rejects IMU messages without orientation covariance or valid quaternions.
   - Always route raw IMU through `imu_filter_madgwick` to generate `/imu/data` before feeding it to `stereo_odometry`.
5. **Stereo Baseline & Optical Frames**:
   - Baseline between left and right camera optical frames is strictly **$0.12\text{ m}$**.
   - Camera projection matrix `P_right[3]` must strictly match $-f_x \cdot \text{baseline}$.
   - Left and right cameras must have zero vertical offset or relative pitch/roll rotation.

---

## 9. Known Issues & Troubleshooting

- **Left/Right Frame Sync Warnings**:
  - *Symptom*: RTAB-Map prints warnings about stereo timestamps differing by $> 0.02\text{ s}$ or dropping frames.
  - *Mitigation*: Ensure `approx_sync: True` and `approx_sync_max_interval: 0.02` are set in both `stereo_odometry` and `stereo_sync`. Ensure both cameras render synchronously in `runner.py`.
- **IMU Orientation Drift / Inversion**:
  - *Symptom*: Odometry rapidly drifts upward or diverges.
  - *Diagnostic*: Run `src/athena_description/script/check.py` to compare measured IMU gravity vector against base transform. Check `use_mag: False` in `imu_filter_madgwick`.
- **Stereo Correspondences Rejected**:
  - *Symptom*: `/rtabmap/odom_info` reports low inliers (`inliers < 15`) or `lost`.
  - *Checklist*: Verify `martian_terrain` texture/lighting in Genesis, verify `Vis/MinInliers: '15'` and `GFTT/QualityLevel: '0.0005'` in `rtabmap_VIO.launch.py`.
- **Rover Flipping on Martian Terrain**:
  - *Symptom*: Complex terrain mesh causes high collision forces or clipping.
  - *Mitigation*: Keep `convexify=False` for terrain in `scene.py`, use `mars_terrain_lowres.stl`, and adjust rover spawn clearance (`SPAWN_CLEARANCE = 1.5` in `config.py`).

---

## 10. Agent Verification Checklist

Before reporting any task complete or committing code:
1. **Topic Rate Check**:
   ```bash
   ros2 topic hz /left/image_rect
   ros2 topic hz /right/image_rect
   ros2 topic hz /imu/data
   ```
2. **Odometry Quality Check**:
   Confirm `/rtabmap/odom` is publishing and quality $> 0$:
   ```bash
   ros2 topic echo /rtabmap/odom_info --field inliers
   ```
3. **No Deprecated Code**:
   Ensure no legacy ORB-SLAM3 artifacts were introduced:
   ```bash
   grep -ri "orb_slam" src/
   ```
4. **Clean Build**:
   ```bash
   colcon build --symlink-install --packages-select athena_description
   ```
