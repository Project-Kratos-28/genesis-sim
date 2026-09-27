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

## Running in a UTM VM

UTM's virtual GPU may not provide a GLX framebuffer configuration that GLFW
can use. This can cause `GLXBadFBConfig` or a segmentation fault when starting Genesis or a GLIM viewer. The mapping node can run without a viewer, but `offline_viewer` requires a graphical desktop.

From an Ubuntu desktop terminal, source the helper once in each shell:

```bash
cd ~/genesis-sim
source scripts/utm_glim_env.sh
```

The helper configures the Xwayland display and forces Mesa software rendering.
It also removes `MESA_GL_VERSION_OVERRIDE`, which can cause an incompatible
framebuffer configuration on this setup. Verify the renderer if needed:

```bash
sudo apt install mesa-utils
glxinfo -B
```

The renderer should normally be `llvmpipe`.

Run the simulator headlessly:

```bash
export GENESIS_SHOW_VIEWER=0
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch athena_description display.launch.py
```

In another Ubuntu desktop terminal, source the helper again before starting
GLIM:

```bash
cd ~/genesis-sim
source scripts/utm_glim_env.sh
source /opt/ros/jazzy/setup.bash
ros2 run glim_ros glim_rosnode \
  --ros-args \
  -p config_path:=$HOME/glim_config \
  -p use_sim_time:=true
```

To open a saved map in the graphical viewer:

```bash
source scripts/utm_glim_env.sh
ros2 run glim_ros offline_viewer
```

These display and Mesa variables only need to be sourced once per shell. To apply them automatically to every interactive Bash shell, add this line to `~/.bashrc` inside the VM:

```bash
source "$HOME/genesis-sim/scripts/utm_glim_env.sh"
```

Do not add that line on the host macOS machine or on a normal Linux system with a working accelerated OpenGL configuration. The UTM workaround is a VM/display compatibility workaround, not a GLIM or ROS requirement.

## Everyday usage

Each new terminal session:

```bash
cd genesis-sim
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```

To apply them automatically to every interactive Bash shell, add the source commands to `~/.bashrc`.
