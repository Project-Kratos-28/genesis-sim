# AGENTS.md — Kratos Workspace Guide

Welcome to the **Kratos** repository. This document serves as the authoritative operational manual and architectural guide for AI agents and developers working in this workspace.

---

## 1. Project Overview

**Kratos** is a ROS 2 Jazzy robotics workspace hosting the **Athena Rover** (`athena_rover`), a 4-wheeled planetary rover designed for rough Martian terrain exploration. The workspace pairs a GPU-accelerated Genesis physics simulation with modular, runtime-switchable rover kinematics.

### Key Capabilities & Technologies
- **Robotic Platform**: Athena Rover — 4-wheel drive, 4-wheel independent steering rover with dual bogey suspension.
- **Physics Simulation**: [Genesis](https://genesis-world.readthedocs.io/) GPU-accelerated physical simulation (`gs.init(backend=gs.gpu)`), running a custom Martian elevation terrain mesh.
- **Kinematics Strategies**: Modular runtime-switchable steering controllers:
  - **Ackermann Steering** (front-wheel steering, rear wheels locked straight)
  - **Differential Drive** (skid-steer mode, all steer joints at 0 rad)
  - **Independent 4-Wheel Steering** (all-wheel steer, crab walking, point turns)
- **Target OS / ROS Distribution**: Ubuntu 24.04 LTS with ROS 2 Jazzy Jalisco.

> **Scope note — simulated perception, no physical camera.** The workspace is a physics
> sandbox plus a **fully simulated ZED2i**: `genesis_bridge/zed_sensors.py` renders a
> Genesis stereo pair and samples a Genesis IMU, then publishes them under the **exact topic
> names, frames and HD720 intrinsics of the real ZED ROS 2 wrapper** (`/zed/zed_node/...`).
> `launch/rtabmap_vio.launch.py` runs `imu_filter_madgwick` + RTAB-Map `stereo_odometry` on
> those topics, so a VIO configuration validated here transfers unchanged to a physical
> ZED2i + RTAB-Map.
>
> A simulated **Livox MID-360** (`genesis_bridge/livox_lidar.py`) ray-casts the same scene
> from the URDF `livox_mid360_link` frame and publishes `sensor_msgs/PointCloud2` on
> `/livox/lidar` in frame `livox_mid360_link` — the topic `livox_ros_driver2` uses for its
> `xfer_format:=1` output. Nothing is published by the bridge except `/clock`,
> `map → base_footprint`, the ZED sensor stream and that point cloud; the bridge **never**
> publishes `/odom` or a `odom` TF.
>
> The ZED2i model in `athena_rover-6.urdf` (`zed_camera_link` + stereo/IMU frames, visual
> mesh only) is both the TF tree and the source of truth for the sensor mount: the render
> cameras sit on the URDF optical-frame origins. See §7 for the mount/orientation proof.
>
> `src/zed-ros2-wrapper`, `src/vision_opencv` and the ZED SDK `.run` installer at the
> workspace root are still present so hardware/perception work can be re-added later, but
> **nothing in this workspace builds against, launches, or imports them**.

---

## 2. System Architecture & Data Flow

```
                      +---------------------------------------+
                      |       Genesis Physics Engine          |
                      |  - Martian Terrain Mesh               |
                      |  - Visual landmark boulders           |
                      |  - Athena Rover URDF Dynamics         |
                      |  - PD joint control (steer + wheels)  |
                      |  - Stereo cameras + IMU sensors       |
                      |  - 360° LiDAR raycast                |
                      +-------------------+-------------------+
                                          |
                                          v
                      +---------------------------------------+
                      |    genesis_bridge (SimulationRunner)  |
                      |  - bridge_node: /cmd_vel & /steering  |
                      |  - rover_controller: PD joint gains   |
                      |  - zed_sensors: 2x camera + IMU PUB   |
                      |  - livox_lidar: PointCloud2 PUB       |
                      |  - publishers: /clock + TF            |
                      +-------------------+-------------------+
                                          |
              /clock · TF: map -> base_footprint · /zed/zed_node/{left,right}/image_rect_color
              + camera_info · /zed/zed_node/imu/data_raw · /livox/lidar
                                          |
                                          v
                      +---------------------------------------+
                      |  imu_filter_madgwick -> imu/data      |
                      |  rtabmap_odom/stereo_odometry -> /odom|
                      |  (rtabmap_vio.launch.py, optional)    |
                      +-------------------+-------------------+
                                          |
                                          v
                      +---------------------------------------+
                      |   robot_state_publisher + RViz        |
                      |   (URDF joint tree, use_sim_time)     |
                      +---------------------------------------+
```

**Per-step loop** (`SimulationRunner.step()` in `runner.py`):
1. `bridge_node.compute_wheel_commands()` → steering mode turns `(/cmd_vel)` into
   `(steer_angles, wheel_angular_velocities)`.
2. `rover_controller.apply()` → `control_dofs_position()` on the 4 steer joints and
   `control_dofs_velocity()` on the 4 wheel joints.
3. `scene.step()` advances physics by `SIM_DT`.
4. Publish `/clock`, the `map → base_footprint` transform from `rover.get_pos()/get_quat()`,
   and whatever ZED sensor samples are due at the new sim time (`zed.publish(...)`).
5. `rclpy.spin_once(node, timeout_sec=0)` services callbacks; repeat.

Before the loop, `SimulationRunner._settle()` steps the physics `SETTLE_STEPS` times with
zero wheel command **without publishing anything** — the rover spawns above the terrain and
the free fall would otherwise be seen by the simulated IMU as 0 g (breaking Madgwick's
gravity initialisation). `/clock` therefore starts at the settled state.

---

## 3. Directory Structure

```
kratos/
├── AGENTS.md                                   # This agent and developer operational guide
├── kratos                                      # Shell helper: activates python venv & ROS 2 Jazzy
├── .gitignore                                  # Git exclusion rules
├── .gitmodules                                 # Submodule mappings (retained, currently unused)
│
├── src/
│   ├── athena_description/                     # Primary (and only built) robot package
│   │   ├── CMakeLists.txt                      # Ament CMake build configuration
│   │   ├── package.xml                         # ROS 2 package dependencies
│   │   │
│   │   ├── launch/
│   │   │   ├── display.launch.py               # Main launcher: robot_state_publisher +
│   │   │   │                                   #   joint_state_publisher_gui + genesis_launcher.py
│   │   │   └── rtabmap_vio.launch.py           # display.launch.py + imu_filter_madgwick +
│   │   │                                       #   rtabmap_odom/stereo_odometry + rtabmap_viz
│   │   │
│   │   ├── img/
│   │   │   └── ground_noise.png                # Noise texture asset (unused at runtime)
│   │   │
│   │   ├── meshes/                             # STL 3D models for visualization & collision
│   │   │   ├── visual/                         # chassis, bogey_arm, wheel, stearing, SteeringMotorHousing,
│   │   │   │                                   #   zed2i (ZED2i camera body, copied from zed_description)
│   │   │   └── collision/                      # Collision meshes for the physics engine
│   │   │
│   │   ├── script/
│   │   │   ├── genesis_launcher.py             # Thin executable entrypoint → genesis_bridge.runner.main
│   │   │   ├── Ackermann.py                    # Standalone AckermannDriveStamped → Twist helper
│   │   │   │                                   #   (not installed, not launched; needs ackermann_msgs)
│   │   │   │
│   │   │   ├── genesis_bridge/                 # Modular Genesis ↔ ROS 2 simulation bridge
│   │   │   │   ├── config.py                   # Constants: assets, sim, gains, sensors, frames, topics
│   │   │   │   ├── runner.py                   # SimulationRunner: settle pre-roll + physics loop
│   │   │   │   ├── scene.py                    # Genesis scene: terrain + landmark boulders + rover URDF
│   │   │   │   ├── bridge_node.py              # GenesisBridge Node: /cmd_vel & /steering_mode
│   │   │   │   ├── rover_controller.py         # DOF index lookup, PD gains, actuator commands
│   │   │   │   ├── zed_sensors.py              # Simulated ZED2i: stereo pair + IMU → ZED topics
│   │   │   │   ├── livox_lidar.py              # Simulated Livox MID-360: raycast → /livox/lidar
│   │   │   │   └── publishers.py               # /clock publisher + map→base_footprint TF broadcaster
│   │   │   │
│   │   │   └── steering/                       # Modular wheel kinematics engine
│   │   │       ├── __init__.py                 # make_steering_mode(name) factory
│   │   │       ├── base.py                     # Abstract SteeringMode interface
│   │   │       ├── wheel_config.py             # Wheel positions, joint names, radius, limits
│   │   │       ├── ackermann.py                # Front-steer, rear-locked
│   │   │       ├── differential.py             # Skid-steer (steer = 0)
│   │   │       └── independent.py              # 4-wheel independent steer
│   │   │
│   │   └── urdf/
│   │       ├── athena_rover-6.urdf             # Plain URDF — loaded by Genesis AND published to
│   │       │                                   #   robot_state_publisher (this is the live model)
│   │       ├── athena_rover-6.urdf.xacro       # Parameterized mirror of the URDF (not launched)
│   │       └── payloads/                       # Simulation terrain assets
│   │           ├── mars_terrain_lowres.stl     # Active terrain STL loaded by scene.py
│   │           ├── martian_terrain.stl         # High-resolution terrain
│   │           ├── martian_terrain_heightfield.npy # NumPy elevation heightfield
│   │           └── model.glb / model.stl / *.blend / *.fbx  # Source assets
│   │
│   ├── vision_opencv/                          # [Submodule] Unused; retained for future perception work
│   └── zed-ros2-wrapper/                       # [Submodule] Unused; retained for future ZED hardware work
│
├── build/                                      # colcon build artifacts (ignored by git)
├── install/                                    # colcon installation tree (ignored by git)
└── log/                                        # colcon build & test logs (ignored by git)
```

> The `genesis_bridge/` package has **no `__init__.py`** — it works as an implicit
> namespace package because `genesis_launcher.py` inserts its own directory into
> `sys.path` before importing `genesis_bridge.runner`.

---

## 4. Environment & Prerequisites

1. **Host Environment**: Ubuntu 24.04 (Linux x86_64), NVIDIA GPU with CUDA drivers installed.
2. **ROS 2 Distribution**: ROS 2 Jazzy Jalisco (`/opt/ros/jazzy/setup.bash`).
3. **Python Virtual Environment**: `~/genesis_env` containing Genesis (`genesis-world`) with
   CUDA-enabled PyTorch. **The simulation must run inside this venv** — `runner.py`/`scene.py`
   import `genesis` at module load, so `ros2 launch` will fail outside it.
   (`scene.py` also imports `trimesh` — already a Genesis dependency — to ray-cast landmark
   heights onto the terrain mesh.)
4. **Required ROS 2 Packages** (bridge only):
   - `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `xacro`
   - Message/TF packages imported by the bridge: `rclpy`, `builtin_interfaces`,
     `geometry_msgs`, `std_msgs`, `rosgraph_msgs`, `nav_msgs`, `tf2_ros`, `sensor_msgs`
     (`nav_msgs` is only needed because `publishers.py` still imports `Odometry` for the
     disabled `/odom` path; `sensor_msgs` carries `Image`, `CameraInfo` and `Imu`)
5. **Additional packages for `rtabmap_vio.launch.py`** (not needed by `display.launch.py`):
   `imu_filter_madgwick`, `rtabmap_odom`, `rtabmap_viz`, `rtabmap_conversions`.
6. **Not required** (even though the submodules exist): `zed_wrapper`, `zed_interfaces`,
   `cv_bridge`, `image_geometry`, `sensor_msgs_py`. The simulated camera publishes raw
   `sensor_msgs/Image` — no ZED SDK, no submodule build.

---

## 5. Build, Sourcing & Running Guide

### 5.1 Environment Sourcing
Use the root convenience script or run manually:
```bash
# Quick source (venv + ROS + workspace overlay):
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
Launch `robot_state_publisher`, `joint_state_publisher_gui` and the Genesis bridge:
```bash
ros2 launch athena_description display.launch.py
```
*Optional argument*: specify the initial steering mode:
```bash
ros2 launch athena_description display.launch.py steering_mode:=ackermann
# Or: steering_mode:=differential, steering_mode:=independent
```
The Genesis viewer window opens (`SHOW_VIEWER = True` in `config.py`; override with
`show_viewer:=false`). The rover spawns at `(10.0, 0.0, 1.7145)` — `SPAWN_Z` derived from
`SPAWN_CLEARANCE` — above the terrain centred at `(10.0, 4.0, 0.0)`.

**The viewer and the simulated cameras compete for the GPU.** Camera reads are ~25x slower
while the viewer window is open (measured 8.9 vs 97.5 steps/s, i.e. 0.09x vs 1.0x realtime),
so sensor/VIO work should run with `show_viewer:=false`.

### 5.4 Visualizing in RViz
There is no committed `.rviz` config anymore:
```bash
rviz2
# Add a "RobotModel" display (Fixed Frame: map).
# TF and /clock come from the bridge; set "Use sim time" in RViz if needed.
```

### 5.5 Teleoperating the Rover
Publish velocity commands to `/cmd_vel`:
```bash
# Drive forward at 0.5 m/s with a 0.2 rad/s turn
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}" -r 10
```

Switch steering mode dynamically at runtime:
```bash
ros2 topic pub /steering_mode std_msgs/msg/String "{data: 'independent'}" --once
```

### 5.6 Simulated ZED2i & VIO
`display.launch.py` already publishes the simulated camera/IMU stream (nothing extra to start
for the sensors alone). To run stereo-inertial odometry on it:
```bash
ros2 launch athena_description rtabmap_vio.launch.py
# unanchored odometry (odom frame free, as on hardware):
ros2 launch athena_description rtabmap_vio.launch.py ground_truth_frame_id:=''
```
That launches `display.launch.py` plus:

| Node | Role |
| :--- | :--- |
| `imu_filter_madgwick` | `imu/data_raw` → `imu/data` (orientation), same as on hardware |
| `stereo_odometry` (`rtabmap_odom`) | stereo VIO; publishes `/odom`, `publish_tf:=false` |
| `rtabmap_viz` | live feature/odometry view |

Key points:
- `show_viewer` (default `true`) opens the Genesis window next to `rtabmap_viz`, so both can be
  watched at once. Keep in mind the viewer starves the camera reads (§5.3): camera images drop
  to ~0.09x realtime. Pass `show_viewer:=false` when you need VIO accuracy or a long drive.
- `publish_tf:=false` — the bridge keeps sole ownership of `map → base_footprint`; the
  estimate is read from `/odom` instead of competing for the TF tree.
- `frame_id: base_footprint` — RTAB-Map publishes `odom_frame_id → frame_id`, so this sets the
  convention of every `/odom` pose. Keeping the body frame (+X forward) is what makes the
  RViz Odometry arrow (drawn along pose +X) point where the rover is going. Using the camera's
  optical frame here puts the pose +X 90° to the right of travel.
- `ground_truth_frame_id` defaults to `map` **in this simulation launch**: RTAB-Map resets its
  *initial* odometry pose from `map → base_footprint` (initialization only — it does not
  continuously correct or fuse the truth). This is required because `imu_filter_madgwick` has
  no magnetometer, so its startup yaw is arbitrary; RTAB-Map's `odom` frame inherits that yaw,
  and the static identity `map → odom` would otherwise rotate the whole estimate (and the
  arrow) relative to `map`. On hardware there is no ground truth: pass
  `ground_truth_frame_id:=''`.
- Measured with both fixes: RViz arrow (odom pose +X) vs ground-truth heading **median 0.0°,
  max 0.4°**; `/odom` position tracks the truth TF to 2–4 cm over a short drive; odom yaw
  *change* matches truth to 0.1–0.7°. (The old optical-frame setup measured +115° at the end of
  a straight drive.)
- RTAB-Map parameters (`Odom/*`, `Vis/*`, `GFTT/*`) must be passed as **strings** in the
  parameter dict or the node throws `InvalidParameterTypeException`.
- Measured on this terrain: forward-drive scale error ≈ −0.2…−0.5%, rotation error ≈ 0.1–0.2°
  over 2–3 m. Hard turns at speed flip the rover (see §9), which is a physics problem, not an
  estimator one — keep `|angular.z| ≲ 0.05 rad/s` at 0.25 m/s when you need a long run.

To move the same pipeline to hardware, drop `display.launch.py`, `use_sim_time`,
`publish_tf` and the ground-truth anchor (`ground_truth_frame_id:=''`), and point the
remappings at the real `/zed/zed_node/...` topics — the names are identical. Keep
`frame_id: base_footprint` (or your own body frame): the arrow/pose-convention fix is not
simulation-specific.

### 5.7 Visual landmarks
The terrain STL has no UVs (`Texture given but asset missing uv info`), so it cannot be
textured, and a featureless white mesh gives VIO nothing to track (it ends up locking onto
the rover's own static shadow). `scene.py::_add_visual_landmarks()` therefore scatters
`LANDMARK_COUNT` static, **non-colliding** boulders whose heights are ray-cast onto the
terrain. They add no physics (`collision=False`), but their material is
`gs.materials.Kinematic(use_visual_raycasting=True)`, so they are also part of the Livox
raycast (§5.8) and are visible in `/livox/lidar` — otherwise the LiDAR could only see the
collision geometry (terrain + rover). `Kinematic` (not `Rigid`) matters: it puts the boulders
in the kinematic solver, where every link is fixed, so Genesis builds their visual raycast
BVH once. With `Rigid` they would share the rover's rigid solver, whose movable links force a
full visual BVH rebuild **every physics step** (measured 150 → 27 steps/s). All boulders are
merged into **one** visual mesh (exported to a temp STL) because entity count directly costs
camera render time. Disable with `VISUAL_LANDMARKS = False` in `config.py` (expect VIO to lose
tracking, and no rocks in the scan).

### 5.8 Simulated Livox MID-360 LiDAR
`display.launch.py` also publishes a simulated point cloud; nothing extra needs to start.
`genesis_bridge/livox_lidar.py` ray-casts the scene (terrain + rover collision geometry, plus
the visual landmark boulders via `use_visual_raycasting`) from the URDF `livox_mid360_link`
frame at 10 Hz simulated time and publishes `sensor_msgs/PointCloud2` on `/livox/lidar`:

```bash
ros2 topic hz /livox/lidar          # ~10 Hz sim time
ros2 run rviz2 rviz2                # PointCloud2 display, topic /livox/lidar
```

- **Only published while subscribed**, like the ZED images: `LivoxLidar.publish()` checks
  `get_subscription_count()` and skips the ROS message when nobody listens. Unlike the
  cameras, the raycast itself is owned by Genesis and its BVH is refreshed every physics
  step, so simply *creating* the sensor costs ~2.8 ms/step whether or not anyone reads it
  (≈194 steps/s headless vs ≈478 steps/s without the LiDAR). The landmark boulders add a
  visual BVH on top (≈150 steps/s): it is built once because they use the `Kinematic`
  material, not per step — keep it that way (§5.7). Tune `LIVOX_BEAMS_H/_V` in
  `config.py` (ray count is nearly free; the fixed cost is the BVH refresh).
- **Pattern**: uniform spherical grid over the hardware FOV — 360° azimuth, `-7°…+52°`
  elevation, 0.1 m blind zone, 100 m range. The real unit uses a non-repetitive rosette, so
  density is representative, not bit-identical. Most rays above the horizon legitimately
  return nothing (sky); a typical settled scan is ≈7.4k points (≈6.8k without the landmark
  boulders in the raycast).
- **Frame**: `livox_mid360_link` (`base_footprint → base_link` z +0.155, then
  `livox_mid360_link` at `z = +0.175`; both rpy 0). The cloud is in that frame, so it lines
  up with the URDF without extra transforms. On hardware, run `livox_ros_driver2` with
  `xfer_format:=1` and set its `frame_id` to `livox_mid360_link` (or add the matching static
  transform). For the default `CustomMsg` form, use `livox_ros_driver2/msg/CustomMsg`
  instead — this bridge publishes PointCloud2 only.
- **Fields**: `x, y, z, intensity` (`FLOAT32`, `point_step = 16`). `intensity` is synthetic
  (inverse-square range falloff clipped to 0.05–1.0); treat it as a placeholder.
- **Self-returns**: a 360° sensor at that height sees the rover's own chassis and the ZED2i
  housing on the lowest rings — physically correct. Raise `LIVOX_MIN_RANGE` if your consumer
  needs them gone.

---

## 6. Kinematics & Rover Geometry

All rover geometry values are centralized in
`src/athena_description/script/steering/wheel_config.py`.

### Rover Specifications
- **Wheel Radius**: $R = 0.115\text{ m}$
- **Front Axle X**: $+0.39291\text{ m}$, **Rear Axle X**: $-0.34458\text{ m}$
- **Wheelbase ($L$)**: $0.73749\text{ m}$
- **Front Track ($W_f$)**: $0.78362\text{ m}$ (offsets $y = \pm 0.39181\text{ m}$)
- **Rear Track ($W_r$)**: $0.75796\text{ m}$ (offsets $y = \pm 0.37898\text{ m}$)
- **Max Steer Angle**: $\pm 1.57\text{ rad}$ ($\pm 90^\circ$), clamped in every mode

### Joint Mappings
| Wheel Position | Steer Joint (revolute) | Wheel Spin Joint (continuous) | Position $(x, y)$ |
| :--- | :--- | :--- | :--- |
| **Front Left** | `steer_front_left` | `wheel_front_left_spin` | $(+0.39291, +0.39181)$ |
| **Front Right** | `steer_front_right` | `wheel_front_right_spin` | $(+0.39291, -0.39181)$ |
| **Rear Left** | `steer_rear_left` | `wheel_rear_left_spin` | $(-0.34458, +0.37898)$ |
| **Rear Right** | `steer_rear_right` | `wheel_rear_right_spin` | $(-0.34458, -0.37898)$ |

`WHEEL_ORDER` in `wheel_config.py` fixes the command vector order returned by every mode:
`["front_left", "front_right", "rear_left", "rear_right"]`. The steering modes and
`RoverController` both rely on it — never reorder one without the other.

### Steering Modes
All modes implement `compute(v, wz) -> (steer_angles, wheel_ang_vels)` and are selected by
name through `make_steering_mode()`; an unknown name raises `ValueError`. At **startup** the
bridge logs a warning and falls back to `DEFAULT_STEERING_MODE` (`ackermann`); at **runtime**
(`/steering_mode`) an unknown name is logged and the current mode is kept.

1. **`ackermann`**: Front wheels solve the steering angle from the instantaneous centre of
   curvature (front axle as reference: $\tan\theta = \omega_z \cdot x_{\text{front}} /
   (v - \omega_z y)$); rear wheels stay at $0\text{ rad}$ and take the differential speed
   $v - \omega_z y$. Angles beyond $\pm\pi/2$ are flipped by $\pi$ with the speed negated.
2. **`differential`**: Every steer joint locked at $0\text{ rad}$; wheel speeds
   $v_w = (v - \omega_z y) / R$ give skid-steering.
3. **`independent`**: Each wheel gets its own ICC vector
   $(v_x, v_y) = (v - \omega_z y,\ \omega_z x)$, so every wheel points along its true path —
   supports crab steering, diagonal translation and zero-radius point turns.

### Actuator Control (`rover_controller.py`)
- **Steer joints**: position control with `STEER_KP = 800`, `STEER_KV = 40`; initialised to 0.
- **Wheel joints**: velocity control with `WHEEL_KP = 0`, `WHEEL_KV = 30` (pure damping).

---

## 7. ROS Interface & Topics Contract

### Published by the bridge
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulated time; advances by `SIM_DT` (0.01 s) per physics step, so its real-world rate is however fast the GPU loop runs |
| TF `map → base_footprint` | `tf2_msgs/msg/TFMessage` | Ground-truth rover pose from `rover.get_pos()` / `get_quat()` (Genesis `[w,x,y,z]` converted to ROS `[x,y,z,w]`) |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Simulated 360° LiDAR scan, frame `livox_mid360_link`, `x/y/z/intensity`, 10 Hz sim time; only while subscribed (§7 below) |

### Subscribed by the bridge
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Uses `linear.x` ($v$) and `angular.z` ($\omega_z$); other fields ignored |
| `/steering_mode` | `std_msgs/msg/String` | Switches to `"ackermann"`, `"differential"` or `"independent"` at runtime |

### Published by the bridge — simulated ZED2i (`zed_sensors.py`)
Names, frames and intrinsics match the real ZED ROS 2 wrapper, so VIO configs transfer
unchanged. Rate limits are counted in **simulated** time (10 Hz images, 100 Hz IMU).

**Images are only rendered while subscribed**: `ZedSensors.publish()` checks
`get_subscription_count()` on the image publishers and skips the (expensive) stereo render when
nobody listens. `camera_info` follows the images; the IMU streams unconditionally (it is cheap).
So `display.launch.py` alone costs the same as before the sensor rig was added.

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/zed/zed_node/left/image_rect_color` | `sensor_msgs/msg/Image` | 1280×720 `rgb8`, frame `zed_left_camera_optical_frame` |
| `/zed/zed_node/right/image_rect_color` | `sensor_msgs/msg/Image` | 1280×720 `rgb8`, frame `zed_right_camera_optical_frame` |
| `/zed/zed_node/left/camera_info` | `sensor_msgs/msg/CameraInfo` | `plumb_bob`, `D = 0`, `fx = fy = 527.839`, `cx = 640`, `cy = 360`, `P` right column 0 |
| `/zed/zed_node/right/camera_info` | `sensor_msgs/msg/CameraInfo` | Same `K`; `P[0,3] = -fx · B = -63.341` (`B = 0.12 m`) |
| `/zed/zed_node/imu/data_raw` | `sensor_msgs/msg/Imu` | accel + gyro with bias/noise; **no orientation** (`orientation_covariance[0] = -1`), frame `zed_imu_link` |

`imu/data` is produced by `imu_filter_madgwick` from `imu/data_raw` — the bridge does not
publish it. Geometry is verified: the analytic projection of a known cube overlays the render
to ~1 px, and measured disparity ≈ `fx·B/Z` (21.3 px vs 21.5 px predicted at 2.9 m).

Render cameras sit exactly on the URDF optical-frame origins `(base_link x=+0.245, y=±0.06,
z=+0.110)`. Because that point is 25 mm *inside* the camera housing, the render uses
`ZED_NEAR = 0.03 m` to clip the housing rather than moving the camera forward — frames,
baseline and camera-IMU lever arm therefore stay exactly as modelled.

### Published by the bridge — simulated Livox MID-360 (`livox_lidar.py`)
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 360° scan, frame `livox_mid360_link`; fields `x, y, z, intensity` (`FLOAT32`, `point_step = 16`), 10 Hz simulated time; published only while subscribed |

Geometry: sensor origin at `base_footprint (0, 0, 0.330)` — `base_link` is `z +0.155` and the
URDF joint is `z +0.175` — with axes identical to `base_footprint` (all rpy 0), so no extra
transform is needed beyond the URDF. FOV is 360° × `-7°…+52°`, range 0.1–100 m. The scan
pattern is a uniform spherical grid rather than the MID-360's non-repetitive rosette, so
point density is representative, not identical. Misses (sky) are dropped; a settled scan is
≈7.4k points. About 2k of those land on the landmark-boulder upper hemispheres; they mostly
replace terrain hits that would otherwise be behind the rocks, so the net increase over a
landmark-free scan is only ≈0.6k.

### Not published by the bridge
`/odom` exists in `config.py` (`TOPIC_ODOM`) and `BridgePublishers.publish_odom()` is
implemented, but the publisher creation and the call in `runner.py` are **commented out**.
The bridge never broadcasts `odom` TF either. `/odom` is published only by
`rtabmap_odom/stereo_odometry` when you run `rtabmap_vio.launch.py`:
`odom → base_footprint`, with `publish_tf:=false`. In simulation the odometry pose is
anchored to `map` at initialization (see §5.6), so the first `/odom` pose matches the
bridge's `map → base_footprint` pose; it then drifts as normal odometry does.

### TF Frame Hierarchy (as defined in `athena_rover-6.urdf`)
```
map                                   (broadcast by genesis_bridge)
 └── base_footprint
      └── base_link                   (fixed: base_footprint_to_base, z = 0.155)
           ├── zed_camera_link        (fixed: zed_camera_joint, screw hole at x=+0.255, z=+0.095)
           │    └── zed_camera_center (fixed, z = +0.015)
           │         ├── zed_left_camera_frame        (fixed, x = -0.01, y = +0.06)
           │         │    └── zed_left_camera_optical_frame   (rpy = -pi/2 0 -pi/2)
           │         ├── zed_right_camera_frame       (fixed, x = -0.01, y = -0.06)
           │         │    └── zed_right_camera_optical_frame  (rpy = -pi/2 0 -pi/2)
           │         └── zed_imu_link (fixed, coincident with zed_camera_center)
           ├── bogey_left             (revolute: bogey_left_pivot)
           │    ├── steer_housing_front_left   (fixed)
           │    │    └── steer_coupler_front_left  (revolute: steer_front_left)
           │    │         └── wheel_front_left     (continuous: wheel_front_left_spin)
           │    └── steer_housing_rear_left    (fixed)
           │         └── steer_coupler_rear_left   (revolute: steer_rear_left)
           │              └── wheel_rear_left      (continuous: wheel_rear_left_spin)
           └── bogey_right            (revolute: bogey_right_pivot)
                ├── steer_housing_front_right
                │    └── steer_coupler_front_right (revolute: steer_front_right)
                │         └── wheel_front_right    (continuous: wheel_front_right_spin)
                └── steer_housing_rear_right
                     └── steer_coupler_rear_right  (revolute: steer_rear_right)
                          └── wheel_rear_right     (continuous: wheel_rear_right_spin)
```

---

## 8. Invariants & Hard Rules for Agents

When contributing to this repository, AI agents **MUST** strictly adhere to the following decisions:

1. **Simulated Sensors Only — No Real Camera Driver Stack**:
   - The ZED2i is **simulated**: `genesis_bridge/zed_sensors.py` (Genesis stereo pair + IMU)
     and `launch/rtabmap_vio.launch.py` (`imu_filter_madgwick` + RTAB-Map `stereo_odometry`)
     are first-class parts of this workspace. Do **NOT** delete them.
   - Real drivers are **not** part of this workspace: no `zed_wrapper`, no ZED SDK calls, no
     `cv_bridge` / `image_geometry`, no ORB-SLAM3. Do not re-add them unless explicitly asked.
   - The ZED2i URDF model in `athena_rover-6.urdf` is **visual/TF only**. It must stay
     driver-free: `package://athena_description/meshes/visual/zed2i.stl`, with no
     dependency on `zed_description` / `zed_wrapper` at build or launch time.
   - Do not make the build depend on `zed_wrapper`, `zed_interfaces`, `cv_bridge`,
     `image_geometry` or `rtabmap_*` / `imu_filter_madgwick`. The retained submodules are
     inert, and `rtabmap_*`/`imu_filter_madgwick` are runtime-only, resolved from `/opt/ros`.
   - **Mount and orientation are verified — do not "fix" them.** The ZED2i sits on the
     front top rail at `zed_camera_joint` `xyz="0.255 0 0.095"` (`rpy="0 0 0"`), i.e.
     **centre-line (`y = 0`), at the `+X` end, lens facing `+X`** — the same direction the
     rover travels for positive `/cmd_vel`. Measured in `base_link`:
     camera mesh `x 0.2270…0.2701`, `y −0.0876…+0.0876` (centre −0.00002),
     `z 0.2501…0.2803`; chassis mesh `x −0.270…+0.270`, `y −0.256…+0.253`.
     The lens face is the mesh's `+X` end (lens bezels Ø≈25 mm at `y = ±0.06`, barrels
     `x 0.0086…0.0231`; the `−X` end is the 30 × 11 mm USB-C connector boss). The optical
     frames agree: `rpy="-1.5707963 0 -1.5707963"` maps optical `+Z` to `+X`.
     A top-down render with `+X` pointing to the **right of the image** makes the camera
     look like it is "on the right side facing backwards" — that is an axis convention,
     not a modelling error. Re-check with forward pointing **up** before changing anything.
     Because those lens elements are flush with a flat front plate and an STL carries no
     colour, the front would otherwise render as a blank plate (easily mistaken for the
     back), so `zed_camera_center` also carries two `r = 0.0096` `zed_lens` cylinder
     visuals at `y = ±0.06`, `x = +0.0154`. They are **visual only** — no mass, no
     collision — and exist purely to make the facing direction legible.
2. **Keep the Bridge Modular**:
   - Do **NOT** revert `genesis_launcher.py` into a monolithic script.
   - Simulation lifecycle stays in `src/athena_description/script/genesis_bridge/`.
   - Steering logic stays in `src/athena_description/script/steering/`.
3. **Simulation Time Contract**:
   - Genesis drives simulated time; the bridge is the **only** `/clock` source.
   - Exactly **one** bridge may run at a time. It holds an exclusive `flock` on
     `~/.ros/genesis_bridge.lock`; a second instance exits with a clear error instead of
     publishing a second, differently-offset `/clock` (which makes time jump backwards and
     clears every downstream TF buffer). Do not remove that guard.
   - Every downstream ROS 2 node (e.g. `robot_state_publisher`) **MUST** set
     `use_sim_time: True` — this is already done in `display.launch.py`.
4. **Single TF Authority**:
   - The bridge owns `map → base_footprint`; `robot_state_publisher` owns the joint tree
     below `base_footprint`. Never broadcast `map → base_footprint` from anywhere else.
   - VIO therefore runs with `publish_tf:=false`. Do **not** let `stereo_odometry` publish
     `odom` TF in this workspace — it would fight the ground-truth tree.
   - `rtabmap_vio.launch.py` publishes a static identity `map → odom` plus RTAB-Map's
     `odom → base_footprint` (from `/odom`). Keep `frame_id: base_footprint` so those poses
     use the rover body convention (see §5.6); the identity `map → odom` only works because
     the simulation launch anchors the initial odometry pose to `map`
     (`ground_truth_frame_id` default). Changing either independently re-introduces the
     90°-rotated arrow.
5. **Sensor Rig Equivalence**:
   - Sensor topics, frame names, intrinsics (`fx = fy = 527.839`, `cx = 640`, `cy = 360`),
     baseline (0.12 m), resolution (1280×720) and frame rates (10 Hz images / 100 Hz IMU)
     mirror the real ZED2i HD720 stream. Changing any of them breaks the "runs unchanged on
     hardware" property — update `config.py` **and** this guide together.
   - The LiDAR side follows the same rule: `/livox/lidar`, frame `livox_mid360_link`,
     PointCloud2 fields `x/y/z/intensity`, 360° × `-7°…+52°`, 0.1–100 m, 10 Hz. The
     simulated pattern is a uniform spherical grid (real unit: non-repetitive rosette), so
     treat density as representative. Keep `LIVOX_*` in `config.py` in sync with this guide
     and with the `livox_mid360_link` joint in `athena_rover-6.urdf`.
   - Sensor poses are the URDF optical-frame origins; keep `ZED_CAMERA_*`/`ZED_IMU_POS` in
     sync with `athena_rover-6.urdf`. `ZED_NEAR` exists only to clip the camera housing.
   - Keep publishing `imu/data_raw` **without** orientation (like the real wrapper) and let
     `imu_filter_madgwick` produce `imu/data`.
6. **URDF Integrity**:
   - `athena_rover-6.urdf` is consumed twice (Genesis + `robot_state_publisher`). It must
     keep exactly one root link (`base_footprint`) and no dangling joint references.
   - `display.launch.py` reads the **plain `.urdf`**, not the `.xacro`; keep the two in sync.

---

## 9. Known Issues & Troubleshooting

- **Rover Flipping on Martian Terrain** *(known, unresolved)*:
  - *Symptom*: The rover occasionally flips or is thrown while driving over the terrain mesh.
    This happens most reliably when **turning under power** on a slope: `angular.z ≈ 0.10`
    at `0.25 m/s` flips it within ~20 s, and VIO then reports `quality=0` and (with
    `publish_null_when_lost`) frozen poses.
  - *Diagnosis*: check the attitude before blaming the estimator —
    `ros2 run tf2_ros tf2_echo map base_footprint` should show roll/pitch of a few degrees,
    not tens. A flipped rover has `|pitch|`/`|roll|` near 90°.
  - *Mitigation*: Keep `convexify=False` for the terrain in `scene.py`, prefer
    `mars_terrain_lowres.stl`, and raise the spawn clearance (`SPAWN_CLEARANCE = 1.5` in
    `config.py`). For validation drives keep `|angular.z| ≲ 0.05 rad/s` at `0.25 m/s`.
- **Body pitching until the bogey hits its ±30° stop** *(fixed — do not regress)*:
  - *Symptom*: On spawn the chassis rotates nose-down 25–30° and a bogey pins at
    `±0.5236 rad`, regardless of terrain.
  - *Cause*: the `bogey_*_pivot` joints are free (damping only, no springs), so the chassis
    is a pendulum about the pivot line at `z = 0`. With `base_link`'s CoM at exactly
    `(0, 0, 0)` that pendulum is in neutral equilibrium; any forward payload (e.g. the
    0.159 kg ZED2i at `x = +0.255 m`) tips it all the way to the joint stop.
  - *Fix*: `base_link`'s inertial origin is set to `(-0.00507, 0, -0.050)` — 50 mm below
    the pivot line for a stable restoring torque, and 5.07 mm aft to cancel the camera
    moment. Keep it in sync with the camera mass/position if either changes.
- **Build warns about `/home/<user>/thirdparty/ORB_SLAM3/.../Sophus/build`**:
  - *Symptom*: `colcon build` prints `The path '.../ORB_SLAM3/Thirdparty/Sophus/build' in the
    environment variable CMAKE_PREFIX_PATH doesn't exist`.
  - *Cause*: Stale `Sophus_DIR` / `CMAKE_PREFIX_PATH` exports in `~/.bashrc` from the removed
    ORB-SLAM3 setup. Harmless; remove those lines from `~/.bashrc` to silence it.
- **`ros2 launch` fails with `ModuleNotFoundError: genesis`**:
  - *Cause*: launched outside the venv. Run `source kratos` (or `source ~/genesis_env/bin/activate`)
    before launching.
- **Madgwick logs "The IMU seems to be in free fall, cannot determine gravity direction!"**
  and VIO starts with a bogus pose *(fixed — do not regress)*:
  - *Cause*: the rover spawns `SPAWN_CLEARANCE` above the terrain; the drop reads ~0 g on the
    IMU while Madgwick is initialising.
  - *Fix*: `SimulationRunner._settle()` steps physics `SETTLE_STEPS` times **before anything is
    published**, so `/clock` and every sensor sample start post-landing.
- **VIO reports `quality=0` / frozen pose on a straight drive**:
  - *Cause*: the terrain STL has no UVs, so an untextured scene leaves nothing to track.
  - *Fix*: keep `VISUAL_LANDMARKS = True` (§5.7). Also confirm the rover has not flipped
    (previous item) and that the IMU stream is flowing (`ros2 topic hz /zed/zed_node/imu/data`).
- **Simulated camera runs at ≪10 Hz / sim time crawls**:
  - *Cause*: the interactive viewer is open. It shares the GPU with the sensor rasterizers,
    making each camera read ~25x slower.
  - *Fix*: `show_viewer:=false` on the launch command (the viewer is on by default in
    `rtabmap_vio.launch.py`). Verify with
    `ros2 topic hz /zed/zed_node/left/image_rect_color` — expect ≈ sim-time 10 Hz.
- **No images arrive at all**:
  - *Cause*: nothing is subscribed. Image rendering is gated on
    `get_subscription_count()` so an idle scene does not pay for stereo rendering.
  - *Fix*: start a consumer (`rtabmap_vio.launch.py`, `rqt_image_view`, ...) — images begin
    immediately.
- **No point cloud on `/livox/lidar`**:
  - *Cause 1*: nothing is subscribed — `LivoxLidar.publish()` is gated the same way the
    cameras are.
  - *Cause 2*: RViz has no `PointCloud2` display, or its fixed frame is not `map` /
    `livox_mid360_link`.
  - *Check*: `ros2 topic info /livox/lidar` (1 publisher, ≥1 subscriber), and
    `ros2 run tf2_ros tf2_echo base_footprint livox_mid360_link` should give
    `[0.000, 0.000, 0.330]`, identity rotation. An almost-empty cloud is normal — most rays
    point at the sky; the settled ground ring is ≈7.4k points (≈6.8k without the visual
    landmark boulders in the raycast).
- **Rocks / boulders are missing from `/livox/lidar`**:
  - *Cause*: the boulders are `collision=False`, so they are not in the collision geometry
    the LiDAR casts against unless they opt in through `use_visual_raycasting`.
  - *Fix*: keep `material=gs.materials.Kinematic(use_visual_raycasting=True)` in
    `scene.py::_add_visual_landmarks()` (§5.7). Using `Rigid` also makes them visible, but it
    moves them into the rover's rigid solver and forces a full visual-BVH rebuild every
    step — measured 150 → 27 steps/s.
  - *Expected*: a settled scan is ≈7.4k points, of which ~2k land on boulder upper
    hemispheres and ~75 of the 160 boulders get at least one return (the rest are out of the
    beam pattern, occluded, or too small at range).
- **Simulation slows down after adding the LiDAR**:
  - *Cause*: Genesis refreshes the raycast BVH every physics step while a `Lidar` sensor
    exists, so the cost is paid even with no subscriber (≈3 ms/step; ray count is nearly
    free). This is the documented trade-off, not a bug.
  - *Fix*: run with `show_viewer:=false`; if you need the step rate back, set
    `LIVOX_BEAMS_H/_V` lower (barely helps) or remove the sensor from `runner.py`.
- **`stereo_odometry` dies with `InvalidParameterTypeException`**:
  - *Cause*: RTAB-Map parameters are declared as strings; passing an int/double fails.
  - *Fix*: quote every `Odom/*`, `Vis/*`, `GFTT/*` value in the launch file.
- **RViz Odometry arrow points 90° away from the rover / `/odom` yaw offset from `map`**
  *(fixed — do not regress)*:
  - *Symptom*: the arrow from RViz's Odometry display on `/odom` points ~90° to the side of
    the direction the rover is actually driving (measured +115° at the end of a straight run).
  - *Cause 1*: `frame_id` was the camera optical frame. RTAB-Map publishes
    `odom_frame_id → frame_id`, so every pose carried the optical convention (x right, y down,
    z forward); RViz draws the arrow along **pose +X**, i.e. the rover's right.
  - *Cause 2*: `imu_filter_madgwick` runs without a magnetometer, so its initial yaw is
    arbitrary (measured ~102–128° while `map` heading was ~−1°). RTAB-Map's `odom` frame
    inherits that yaw, and the static identity `map → odom` then rotates the whole estimate
    relative to `map`.
  - *Fix*: `frame_id: base_footprint` in the launch (`stereo_odometry` **and** `rtabmap_viz`),
    plus `ground_truth_frame_id` defaulting to `map` in the simulation launch, which resets
    only the *initial* odometry pose (RTAB-Map `OdometryROS.cpp`, `odometry_->reset()`).
  - *Verified*: arrow vs ground-truth heading median 0.0°, max 0.4°; odom yaw change matches
    truth to 0.1–0.7°. On hardware pass `ground_truth_frame_id:=''`; keep `base_footprint`.
  - *Diagnose*: `ros2 topic echo /odom --once --field child_frame_id` must print
    `base_footprint` (not `zed_left_camera_optical_frame`), and the log must show
    `Initializing odometry pose ... (from "map" -> "base_footprint")`.
- **`/odom` translation does not match the TF ground truth (direction only, magnitude fine)**:
  - *Cause*: with `ground_truth_frame_id:=''` RTAB-Map's `odom` frame is **gravity-aligned by
    the IMU** at initialisation, not `map`, so absolute pose comparisons show a pure-rotation
    mismatch.
  - *Fix*: leave the simulation default (`ground_truth_frame_id:=map`). For unanchored runs,
    compare frame-invariant quantities (travelled distance, relative rotation) instead.
- **Stale nodes corrupt VIO measurements**:
  - *Symptom*: `ros2 node list` shows duplicates of `/stereo_odometry`, `/genesis_bridge`, etc.
  - *Cause*: a previous launch was not fully killed (launch kills the parent, not always the
    children).
  - *Fix*: verify one of each before measuring. Kill by PID
    (`ps -eo pid,cmd | grep -E "genesis_launcher|stereo_odometry"`); never `pkill -f` with a
    pattern that matches your own shell command (it self-kills).
  - *Second bridge is refused*: `GenesisBridge` holds an exclusive `flock` on
    `~/.ros/genesis_bridge.lock` for its whole lifetime, so starting a second simulation now
    fails immediately with
    `[genesis_bridge] ERROR: another genesis_bridge is already running (pid …)` and the second
    process never publishes `/clock`. The kernel drops the lock when the owning process exits,
    so a crashed or SIGKILLed bridge cannot leave a stale lock behind — no manual cleanup is
    needed. Kill the PID named in the message (or restart the launch) and start again.
- **`Detected jump back in time` / `Clearing TF buffer` / `TF_OLD_DATA` in `stereo_odometry`
  and `rtabmap_viz`, and the RViz point cloud or odometry disappears**:
  - *Cause*: **two simulation bridges publishing `/clock` at once**. Each derives `sim_time`
    from the shared persisted offset (`~/.ros/genesis_bridge_clock.txt`) at its own start
    time, so the two clocks differ by however long the first had been running (measured
    ~19.6 s). Consumers see time jump backwards every other message, flush their TF buffers,
    and can never resolve `map → base_footprint` — the data exists but is not drawn.
  - *Fix*: keep exactly one bridge. The single-instance lock above now prevents this; if the
    warnings still appear, `ros2 topic info /clock` must show `Publisher count: 1`. Clock
    persistence is atomic (temp file + `os.replace`) so a killed bridge cannot leave a
    half-written offset either.
- **Nothing moves on `/cmd_vel`**:
  - *Check*: the bridge logs `Steering mode: <name>` on startup; confirm the mode name is
    valid, and check `WHEEL_KV` / `STEER_KP` in `config.py`.
- **No pose in RViz**:
  - *Check*: Fixed Frame must be `map` (or `base_footprint`); core `tf2_echo map base_footprint`.

---

## 10. Agent Verification Checklist

Before reporting any task complete or committing code:
1. **Bridge Health Check**:
   ```bash
   ros2 topic hz /clock
   ros2 topic info /clock         # expect Publisher count: 1 (a second bridge is refused)
   ros2 topic info /cmd_vel      # expect a subscription from the bridge
   ```
2. **TF Check**:
   ```bash
   ros2 run tf2_ros tf2_echo map base_footprint
   ```
3. **Simulated ZED2i Check** (expect all five topics, images at ~10 Hz sim time):
   ```bash
   ros2 topic list | grep zed
   ros2 topic hz /zed/zed_node/left/image_rect_color
   ros2 topic echo /zed/zed_node/left/camera_info --once --field k   # 527.839 ... 640 ... 360
   ros2 topic echo /zed/zed_node/right/camera_info --once --field p  # p[3] = -63.341
   ros2 topic echo /zed/zed_node/imu/data_raw --once --field orientation_covariance  # [0] = -1
   ```
4. **Simulated Livox MID-360 Check**:
   ```bash
   ros2 topic info /livox/lidar      # 1 publisher, type sensor_msgs/msg/PointCloud2
   ros2 topic hz /livox/lidar        # ~10 Hz sim time once a subscriber exists
   ros2 topic echo /livox/lidar --once --field header.frame_id   # livox_mid360_link
   ros2 topic echo /livox/lidar --once --field width             # ≈7400 (sky returns dropped); ≈6800 without landmark boulders
   ros2 run tf2_ros tf2_echo base_footprint livox_mid360_link    # [0, 0, 0.330], identity
   ```
5. **No Stray Real-Driver Code** (expect no matches outside `zed_sensors.py` /
   `rtabmap_vio.launch.py`, which are the simulated stack):
   ```bash
   grep -rni "zed_wrapper\|zed_interfaces\|cv_bridge\|image_geometry\|rtabmap\|imu_filter" \
     src/athena_description/ --include="*.py" --include="*.xml" | grep -v "_zed\|rtabmap_vio"
   ```
   (`AGENTS.md` and the launch file are the only legitimate mentions.)
6. **URDF Sanity** (expect one root link, 24 links, 23 joints — 23/22 before the Livox
   link — with no duplicates and no dangling refs):
   ```bash
   python3 -c "import xml.etree.ElementTree as ET; \
   r=ET.parse('src/athena_description/urdf/athena_rover-6.urdf').getroot(); \
   print(len(r.findall('link')), len(r.findall('joint')))"
   ```
7. **VIO Smoke Test** (optional, ~1 min):
   ```bash
   ros2 launch athena_description rtabmap_vio.launch.py show_viewer:=false
   # in a second shell: drive straight, then stop
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
   ros2 topic echo /odom --once --field child_frame_id   # base_footprint
   ros2 topic echo /odom --once                          # first pose ≈ map -> base_footprint
   ```
   The log must contain `Initializing odometry pose ... (from "map" -> "base_footprint")`
   and `quality` > 0. RViz's Odometry arrow (fixed frame `map` or `odom`) should point along
   the rover's heading, not 90° to the side.
8. **Clean Build**:
   ```bash
   colcon build --symlink-install --packages-select athena_description
   ```
