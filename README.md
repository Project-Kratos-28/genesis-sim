# genesis-sim

## Prerequisites

- Ubuntu with ROS 2 Jazzy installed (`/opt/ros/jazzy`)
- `python3-venv`:
  ```bash
  sudo apt install python3-venv
  ```
Setup instructions to clone and run the repository:
## 1. Clone the repository

```bash
git clone https://github.com/Project-Kratos-28/genesis-sim.git
cd genesis-sim
```

## 2. Create the virtual environment

Create the venv with `--system-site-packages` so it has access to ROS's
Python packages:

```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
```

Prevent colcon from scanning into the venv:

```bash
touch .venv/COLCON_IGNORE
```

## 3. Install dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

## 4. Build the workspace

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Environment variables (VM / software rendering)

If running on a VM without GPU passthrough, add the
following to `.venv/bin/activate` so they're set automatically on activation:

```bash
echo 'export PYOPENGL_PLATFORM=osmesa' >> .venv/bin/activate
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> .venv/bin/activate
echo 'export MESA_GL_VERSION_OVERRIDE=3.3' >> .venv/bin/activate
```

## Everyday usage

Each new terminal session:

```bash
cd genesis-sim
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```
