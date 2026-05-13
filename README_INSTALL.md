# StarTouch SDK Installation and Runtime Guide

This document describes how to install the StarTouch/FastTouch SDK, build the
Python extension, and run the verified SDK examples.

## Supported System

The current CMake configuration selects a bundled `libstartouch.so` according
to the Linux distribution version. Supported Ubuntu/Debian versions are:

- Ubuntu 20.04
- Ubuntu 22.04
- Ubuntu 24.04 or newer compatible versions

The SDK requires Python 3.8 or newer. Python 3.10 is recommended and is the
environment used by the current project scripts.

## System Dependencies

Install build tools and C++ dependencies:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev liborocos-kdl-dev pybind11-dev
```

Notes:

- `cmake`, `build-essential`, `libeigen3-dev`, and `pybind11-dev` are required
  by the local CMake/pybind11 build.
- `liborocos-kdl-dev` is needed by the kinematics headers used in the SDK
  source tree.
- `Threads` is provided by the system toolchain and is resolved by CMake.

If you use CAN devices, configure and bring up the CAN interface before running
hardware scripts:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```

Adjust `can0`/`can1` and bitrate according to your hardware setup.

## Python Environment

Create and activate a conda environment:

```bash
conda create -n LumosTouch python=3.10 -y
conda activate LumosTouch
python -m pip install --upgrade pip setuptools wheel
```

## Install Python Dependencies and SDK

Run the following commands in the SDK root:

```bash
cd /home/lumos/code/FastTouchV2/fnl/fnl/startouch_sdk
python -m pip install -r requirements.txt
python -m pip install .
```

For development, use editable install:

```bash
python -m pip install -e .
```

The Python dependencies are declared in both `requirements.txt` and
`pyproject.toml`. They include:

- `scikit-build-core`
- `pybind11`
- `packaging>=24`
- `typer`
- `numpy`
- `scipy`
- `PyYAML`
- `python-can>=4.3.1`
- `tqdm`
- `matplotlib`
- `opencv-python`
- `pillow`
- `h5py`

## Optional ROS 2 Dependencies

The LeRobot data collection and teleoperation scripts use ROS 2 Python APIs:

- `rclpy`
- `sensor_msgs`

These are not pip dependencies. Install/source a ROS 2 environment compatible
with your system, then verify imports:

```bash
python -c "import rclpy; from sensor_msgs.msg import JointState"
```

The LeRobot scripts are documented in `README_API.md` as currently unverified
interfaces.

## Optional OpenPI / SAM3 Dependencies

The online policy rollout scripts such as
`interface_py/pi0_rollout_single_startouch_lxh.py` and
`interface_py/pi0_rollout_single_startouch_lxh_waypoints.py` import
`openpi_client` and SAM3 modules. These dependencies are outside the base SDK
requirements and must be installed/provided by the OpenPI/SAM3 runtime
environment before using those scripts.

## Manual CMake Build

Normally `python -m pip install .` builds through `scikit-build-core`. If you
need a manual CMake build:

```bash
cd /home/lumos/code/FastTouchV2/fnl/fnl/startouch_sdk
mkdir -p build
cd build
cmake ..
make -j
cd ..
```

The CMake build writes the `startouch` Python extension to `interface_py/` and
copies the OS-version-specific `libstartouch.so` into `src/libstartouch.so`.

## Basic Verification

Passive joint reading, without active command:

```bash
cd /home/lumos/code/FastTouchV2/fnl/fnl/startouch_sdk
python interface_py/read_joints_passive.py
```

Hardware connectivity test:

```bash
python interface_py/test_hardware.py
```

MoveJ / MotionProgram test:

```bash
python interface_py/test_motion_program_movej.py
```

MoveP Cartesian test:

```bash
python interface_py/test_movep_cartesian.py
```

Interactive IK / dual-arm example:

```bash
python interface_py/ik.py
```

## Creating Arm Instances

After creating a `SingleArm`, keep a short delay before issuing motion commands
so the controller can initialize and move/settle safely.

Single arm:

```python
import time
from startouchclass import SingleArm

arm_controller = SingleArm(can_interface_="can0", enable_fd_=False)
time.sleep(2)
```

Dual arm:

```python
import time
from startouchclass import SingleArm

arm_right = SingleArm(can_interface_="can0", enable_fd_=False)
arm_left = SingleArm(can_interface_="can1", enable_fd_=False)
time.sleep(2)
```

Current scripts use `enable_fd_=False`; CAN FD is not part of the validated
runtime path.

## Replay Configuration

The current MoveP replay scripts read this config file:

```text
interface_py/replay_refresh_config.yaml
```

They do not read `config.yaml` from the current working directory.

Important fields:

```yaml
DATA_ROOT: "/home/lumos/code/FastTouchV2/fnl/fnl/fastumi/DATA"
dual_multi_session_dir: "/home/lumos/code/FastTouchV2/fnl/fnl/fastumi/DATA/multi_session_20260430"
speed_rate: 1
initial_joints: [0, 0.1, -0.1, 0.1, 0, 0]

StarTouch:
    enable: true

SingleArm:
    single_port: "can0"
    T_base2local: ...

DualArm:
    left_can_port: "can0"
    right_can_port: "can1"
    left_T_base2local: ...
    right_T_base2local: ...
```

`T_base2local` is a standard homogeneous transform:

```text
T_base_pose = T_base2local @ T_recorded_pose
```

The replay scripts align gripper data by pose timestamps and send gripper
targets from a separate sync thread while MoveP is executing.

## Single-Arm Replay

```bash
cd /home/lumos/code/FastTouchV2/fnl/fnl/startouch_sdk
python interface_py/replay_movep.py
```

The script asks you to select a `multi_session*` directory and then a
`session_*` directory. It reads:

```text
session_*/Merged_Trajectory/merged_trajectory.txt
session_*/Clamp_Data/clamp_data_tum.txt
```

## Dual-Arm Replay

```bash
cd /home/lumos/code/FastTouchV2/fnl/fnl/startouch_sdk
python interface_py/replay_movep_dual.py
```

The script reads sessions under `dual_multi_session_dir` and matches:

```text
left_hand_*/Merged_Trajectory/merged_trajectory.txt
left_hand_*/Clamp_Data/clamp_data_tum.txt
right_hand_*/Merged_Trajectory/merged_trajectory.txt
right_hand_*/Clamp_Data/clamp_data_tum.txt
```

Dual-arm replay runs both arms concurrently. If either arm fails, the script
waits for worker threads to exit before cleaning up controller resources.

## Common Issues

### `ModuleNotFoundError`

Confirm that the conda environment is active and the SDK is installed:

```bash
conda activate LumosTouch
cd /home/lumos/code/FastTouchV2/fnl/fnl/startouch_sdk
python -m pip install -r requirements.txt
python -m pip install .
```

### Missing ROS 2 Modules

If `rclpy` or `sensor_msgs` cannot be imported, source your ROS 2 environment
or install the matching ROS 2 packages for your Ubuntu version.

### Replay Config Not Found

Current replay scripts read:

```text
interface_py/replay_refresh_config.yaml
```

They do not use the legacy `Replay_refresh/config.yaml`.

### IK Residual Too Large

This usually means the target pose is unreachable or the coordinate transform is
incorrect. Check:

- `SingleArm.T_base2local`
- `DualArm.left_T_base2local`
- `DualArm.right_T_base2local`
- whether the recorded coordinate frame and robot TCP frame require an
  additional fixed transform

### Requested `time_sec` Too Short

For time-mode waypoint trajectories, the requested duration must be long enough
for the number of trajectory samples and control frequency. If this still
occurs in replay, check the trajectory point count and `MOVE_P_CONTROL_HZ`.
