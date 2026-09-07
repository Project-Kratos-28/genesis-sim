# genesis-sim

## Prerequisites

- Ubuntu with ROS 2 Jazzy installed (`/opt/ros/jazzy`)
- `python3-rosdep`
- `python3-venv`:
  ```bash
  sudo apt install python3-rosdep python3-venv
  ```

## Installation and setup

### 1. Install ROS dependencies

After cloning the repository, run the following command from its root to
resolve and install the ROS dependencies declared in `package.xml`:

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

If `rosdep` has not been initialized on the machine, run the following once
before `rosdep update`:

```bash
sudo rosdep init
```

The ROS packages installed by this project are:

- `ament_index_python`
- `ackermann_msgs`
- `builtin_interfaces`
- `cv_bridge`
- `geometry_msgs`
- `joint_state_publisher_gui`
- `launch`
- `launch_ros`
- `rclpy`
- `robot_state_publisher`
- `rosgraph_msgs`
- `rtabmap_odom`
- `rviz2`
- `sensor_msgs`
- `zed_interfaces`
- `zed_wrapper`
- `xacro`

`rosdep` maps these ROS package names to the appropriate system packages for
the installed ROS distribution, so they do not need to be installed separately
with individual `apt install` commands. The `rosdep` command must be run after
cloning the repository and from the repository root.

### 2. Clone the repository

```bash
git clone https://github.com/Project-Kratos-28/genesis-sim.git
cd genesis-sim
```

### 3. Create the virtual environment

Create the virtual environment with `--system-site-packages` so it can
access ROS Python packages:

```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
```

Prevent colcon from scanning the virtual environment:

```bash
touch .venv/COLCON_IGNORE
```

### 4. Install Python dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### 5. Build the workspace

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Running the display

After setup, source the ROS, virtual environment, and workspace environments
in each new terminal session:

```bash
cd genesis-sim
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```

Launch:

```bash
ros2 launch athena_description display.launch.py
```

## Virtual machines without GPU passthrough

If you are running in a virtual machine without GPU passthrough, add the
following environment variables to `.venv/bin/activate`:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export PYOPENGL_PLATFORM=glx
```

These variables are applied whenever the virtual environment is activated.

## Everyday usage

Each new terminal session:

```bash
cd genesis-sim
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```
