# genesis-sim

## Prerequisites

- Ubuntu with ROS 2 Jazzy installed (`/opt/ros/jazzy`)
- `python3-venv`:
```bash
  sudo apt install python3-venv
```

## Installation and setup

### 1. Install ROS dependencies

Install the ROS message and mapping packages required by the workspace:

```bash
sudo apt update
sudo apt install ros-jazzy-ackermann-msgs ros-jazzy-rtabmap-ros
```

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
