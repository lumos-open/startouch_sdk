# StarTouch Python SDK API

This document summarizes the common Python APIs exposed by
`interface_py/startouchclass.py`. Unless otherwise noted, joint angles and
Euler angles are in radians, Cartesian positions and gripper distances are in
meters.

## Initialization

```python
from startouchclass import SingleArm, MotionProgram

arm = SingleArm(can_interface_="can0", gripper=True, enable_fd_=False)
```

Arguments:

- `can_interface_`: CAN interface name, for example `"can0"`.
- `gripper`: whether the gripper exists and should be enabled.
- `enable_fd_`: whether to enable CAN FD.

Release resources after use:

```python
arm.cleanup()
```

## Gripper APIs

### `setGripperPosition(position)`

Set gripper opening by normalized position.

```python
arm.setGripperPosition(1.0)  # fully open
arm.setGripperPosition(0.0)  # fully closed
```

- `position`: normalized opening ratio, typically `0.0 ~ 1.0`.
- `0.0`: closed.
- `1.0`: open to max distance.

The lower layer clamps the value to `[0, 1]` and maps it to
`position * gripper_dis_max`.

### `setGripperPosition_raw(position)`

Raw gripper position command.

```python
arm.setGripperPosition_raw(0.5)
```

The value is also a normalized opening ratio. Use this interface carefully
because it is intended for lower-level command passthrough.

### `setGripperDistance(distance, kp=None, kd=None)`

Set gripper opening by physical distance.

```python
arm.setGripperDistance(0.085)
arm.setGripperDistance(0.0, 10, 0.5)
```

- `distance`: gripper opening distance in meters, normally `0.0 ~ 0.085`.
- `kp`: optional gripper control stiffness.
- `kd`: optional gripper control damping.

The lower layer clamps the distance to the valid gripper range.

### `setGripperDistance_raw(distance, kp=8.0, kd=0.1)`

Currently this Python wrapper calls `setGripperDistance(distance, kp, kd)`.

### `get_gripper_position()`

Read normalized gripper opening.

```python
g = arm.get_gripper_position()
```

Returns a `float`, typically in `0.0 ~ 1.0`.

### `get_gripper_distance()`

Read gripper opening distance.

```python
d = arm.get_gripper_distance()
```

Returns a `float` in meters.

### `openGripper()` / `closeGripper()`

Convenience APIs for opening and closing the gripper.

```python
arm.openGripper()
arm.closeGripper()
```

## Joint State APIs

### `get_joint_positions()`

Read current joint positions.

```python
q = arm.get_joint_positions()
```

Returns an `np.ndarray` with shape `(6,)`, in radians.

### `get_joint_velocities()`

Read current joint velocities.

```python
dq = arm.get_joint_velocities()
```

Returns an `np.ndarray` with shape `(6,)`, in radians per second.

### `get_joint_torques()`

Read current joint torques.

```python
tau = arm.get_joint_torques()
```

Returns an `np.ndarray` with shape `(6,)`.

## Single Target Joint Control

### `set_joint(positions, tf=2.0, ctrl_hz=400.0)`

Move to one joint target with time planning.

```python
arm.set_joint([0, 0.2, -0.4, 0.5, 0, 0], tf=3.0)
```

- `positions`: target 6-DoF joint position, in radians.
- `tf`: target duration in seconds, must be positive.
- `ctrl_hz`: reserved in the Python wrapper; the current lower-level
  `set_joint` binding uses `positions` and `tf`.

Returns `True`.

### `set_joint_raw(positions, velocities)`

Raw joint command.

```python
arm.set_joint_raw(q_target, dq_target)
```

- `positions`: target 6-DoF joint position, in radians.
- `velocities`: target 6-DoF joint velocity.

This API does not provide high-level trajectory planning. Use it only when the
caller controls command continuity, timing, and safety.

## Joint Waypoint Trajectory

### `move_joint_waypoints(waypoints, time_sec=0.0, speed_percent=-1.0, ctrl_hz=400.0)`

Execute a joint waypoint trajectory.

```python
waypoints = [
    [0, 0.1, -0.3, 0.4, 0, 0],
    [0, 0.2, -0.5, 0.6, 0, 0],
]

duration = arm.move_joint_waypoints(
    waypoints,
    speed_percent=0.3,
    ctrl_hz=400.0,
)
```

- `waypoints`: `(N, 6)` joint waypoints, in radians.
- `time_sec`: requested duration for the whole trajectory, in seconds.
- `speed_percent`: velocity scaling ratio in `(0, 1]`.
- `ctrl_hz`: trajectory/control sampling frequency, default `400 Hz`.

Rules:

- `time_sec` and `speed_percent` are mutually exclusive.
- If neither is provided, the lower layer uses its default speed ratio.
- `waypoints` must not be empty, and every waypoint must contain 6 values.

Returns the planned trajectory duration in seconds.

Blocking behavior:

- The call blocks until the waypoint motion finishes or is interrupted by an
  error.

Lower-level behavior:

- Uses the current joint state as the trajectory start.
- Clamps target joint values to URDF joint limits.
- Runs trajectory safety checks.
- May slow down on safety warnings in speed mode.
- May attempt to insert singularity-avoidance waypoints.

## Cartesian Pose APIs

Cartesian poses use this format:

```python
[x, y, z, roll, pitch, yaw]
```

- `x, y, z`: meters.
- `roll, pitch, yaw`: radians.

### `move_pose_waypoints(...)`

Plan IK for each Cartesian pose and execute the resulting joint waypoints.

```python
poses = [
    [0.3, 0.0, 0.2, 0, 0, 0],
    [0.32, 0.0, 0.2, 0, 0, 0],
]

duration = arm.move_pose_waypoints(poses, speed_percent=0.2)
```

Signature:

```python
arm.move_pose_waypoints(
    poses,
    time_sec=0.0,
    speed_percent=-1.0,
    ctrl_hz=400.0,
    position_tolerance_m=0.005,
    orientation_tolerance_rad=0.05,
)
```

Behavior:

- Solves IK for each pose.
- Uses each IK result as the seed for the next pose.
- Checks FK residual against position and orientation tolerances.
- Executes through the same joint waypoint trajectory backend.

### `move_l(...)`

Execute Cartesian linear segments.

```python
duration = arm.move_l(
    poses,
    speed_percent=0.2,
    blend_radius_m=0.0,
)
```

Signature:

```python
arm.move_l(
    poses,
    time_sec=0.0,
    speed_percent=-1.0,
    blend_radius_m=0.0,
    ctrl_hz=400.0,
    position_tolerance_m=0.003,
    orientation_tolerance_rad=0.05,
)
```

`move_l` plans Cartesian linear segments, converts them to joint waypoints, and
executes the joint waypoint trajectory.

### `move_p(...)`

Execute a Cartesian path with optional blending.

```python
duration = arm.move_p(
    poses,
    speed_percent=0.2,
    blend_radius_m=0.002,
)
```

Signature:

```python
arm.move_p(
    poses,
    time_sec=0.0,
    speed_percent=-1.0,
    blend_radius_m=0.002,
    ctrl_hz=400.0,
    position_tolerance_m=0.003,
    orientation_tolerance_rad=0.05,
)
```

`move_p` is intended for multi-point Cartesian paths and smooth transitions.
The backend converts the Cartesian path to joint waypoints before execution.

## End-Effector Pose Set/Get

### `set_end_effector_pose_euler(pos, euler, tf=2.0)`

Set target end-effector pose by position and Euler angles.

```python
arm.set_end_effector_pose_euler(
    pos=[0.3, 0.0, 0.2],
    euler=[0.0, 0.0, 0.0],
    tf=2.0,
)
```

### `set_end_effector_pose_euler_raw(pos, euler)`

Raw end-effector pose command.

```python
arm.set_end_effector_pose_euler_raw(
    pos=[0.3, 0.0, 0.2],
    euler=[0.0, 0.0, 0.0],
)
```

This interface is intended for passthrough/servo-style control.

### `set_end_effector_pose_quat(pos, quat, tf=2.0)`

Set target end-effector pose by quaternion.

```python
arm.set_end_effector_pose_quat(
    pos=[0.3, 0.0, 0.2],
    quat=[1.0, 0.0, 0.0, 0.0],
    tf=2.0,
)
```

Quaternion format is `[w, x, y, z]`.

### `get_ee_pose_euler()`

Read current end-effector pose.

```python
pos, euler = arm.get_ee_pose_euler()
```

- `pos`: `(3,)`, meters.
- `euler`: `(3,)`, radians.

### `get_ee_pose_quat()`

Read current end-effector pose with quaternion orientation.

```python
pos, quat = arm.get_ee_pose_quat()
```

- `pos`: `(3,)`, meters.
- `quat`: `[w, x, y, z]`.

## IK

### `solve_ik(pos, quat, q_seed=None)`

```python
q, ok = arm.solve_ik(
    pos=[0.3, 0.0, 0.2],
    quat=[1.0, 0.0, 0.0, 0.0],
    q_seed=arm.get_joint_positions(),
)
```

- `pos`: length 3, meters.
- `quat`: length 4, `[w, x, y, z]`.
- `q_seed`: optional length-6 seed, in radians.

Returns:

- `q`: IK solution.
- `ok`: whether IK succeeded.

## MotionProgram

`MotionProgram` combines multiple motion items and executes them through
`run_motion_program()`.

```python
program = MotionProgram()
program.movej([[0, 0, 0, 0, 0, 0]], speed_percent=0.3)
program.movej([[0, 0.2, -0.4, 0.5, 0, 0]], speed_percent=0.3)
program.sleep(1.0)
program.movep(
    [
        [0.3, 0.0, 0.2, 0, 0, 0],
        [0.32, 0.0, 0.2, 0, 0, 0],
    ],
    speed_percent=0.2,
    blend_radius_m=0.002,
)

duration = arm.run_motion_program(program)
```

### `MotionProgram.movej(waypoints, time_sec=0.0, speed_percent=-1.0)`

Append a joint waypoint motion.

### `MotionProgram.movel(poses, time_sec=0.0, speed_percent=-1.0, blend_radius_m=0.0)`

Append a Cartesian linear motion.

### `MotionProgram.movep(poses, time_sec=0.0, speed_percent=-1.0, blend_radius_m=0.002)`

Append a Cartesian path motion.

### `MotionProgram.sleep(sleep_sec)`

Append a blocking sleep item.

```python
program.sleep(1.0)
```

`sleep()` is a hard breakpoint. Pending motion is flushed and completed before
sleep starts.

### `run_motion_program(program, ctrl_hz=400.0)`

Execute a `MotionProgram`.

```python
duration = arm.run_motion_program(program)
```

Returns the accumulated planned motion duration in seconds. The returned value
does not include `sleep()` time.

Blocking behavior:

- The call blocks until all motion items and sleep items finish.

Merge rules:

- Consecutive same-type motions may be merged:
  - consecutive `movej`
  - consecutive `movel`
  - consecutive `movep`
- Different motion types are not merged.
- `sleep()` flushes pending motion.
- In time mode, merged same-type motions accumulate `time_sec`.
- In speed mode, merged same-type motions must use the same `speed_percent`.
- For `movel` and `movep`, `blend_radius_m` must also match for merging.

## Time Mode and Speed Mode

Waypoint, Cartesian, and MotionProgram motion APIs support either time mode:

```python
time_sec=2.0
```

or speed mode:

```python
speed_percent=0.3
```

They are mutually exclusive:

```python
# Invalid
arm.move_joint_waypoints(waypoints, time_sec=2.0, speed_percent=0.3)
```

Recommendations:

- Use `time_sec` when motion must align with external timestamps.
- Use `speed_percent` for relative-speed control and initial hardware tests.
- For first hardware tests, use a conservative `speed_percent`, for example
  `0.1 ~ 0.3`.

## Units and Formats

| Data | Format | Unit |
| --- | --- | --- |
| Joint position | `[q1, q2, q3, q4, q5, q6]` | rad |
| Joint velocity | `[dq1, ..., dq6]` | rad/s |
| Cartesian pose | `[x, y, z, roll, pitch, yaw]` | m, rad |
| Quaternion | `[w, x, y, z]` | unit quaternion |
| Gripper position | `0.0 ~ 1.0` | ratio |
| Gripper distance | `0.0 ~ 0.085` | m |
| `time_sec` / `tf` | scalar | s |
| `ctrl_hz` | scalar | Hz |

## Safety Notes

- Raw joint and raw end-effector APIs do not provide high-level trajectory
  planning. The caller must ensure command continuity, timing, and safety.
- `move_joint_waypoints()`, `move_l()`, `move_p()`, and
  `run_motion_program()` are blocking calls.
- `time_sec` and `speed_percent` cannot both be set.
- Cartesian APIs depend on IK; unreachable targets or excessive residuals
  raise exceptions.
- Gripper APIs clamp out-of-range values, but callers should still pass
  reasonable commands.
- Before hardware execution, verify CAN setup, emergency stop, workspace
  clearance, and initial robot pose.

## LeRobot Data Collection and Teleoperation APIs

Status: **not yet verified on the current hardware/software setup**.

The scripts below are ROS 2 `JointState` bridges for LeRobot-style data
collection, teleoperation, and inference. They require ROS 2 Python packages
(`rclpy`, `sensor_msgs`) in addition to the base SDK dependencies. Treat this
section as API documentation for the current scripts, not as a validated
runtime procedure.

### Single-Arm Master/Follower Teleoperation

Script:

```bash
python interface_py/lerobot_single_arm_tele.py
```

Current behavior:

- Creates two `SingleArm` instances:
  - master/controller arm on `can0`
  - follower arm on `can1`
- Reads master joint positions, joint velocities, and gripper position.
- Sends filtered raw joint commands to the follower with
  `set_joint_raw(...)`.
- Sends gripper passthrough commands to the follower with
  `setGripperPosition_raw(...)`.
- Publishes ROS 2 `JointState` messages:
  - `/left_arm/joint_states_target`
  - `/left_arm/joint_states_now`

Important notes:

- The script currently names the ROS node as a right-arm publisher while
  publishing `/left_arm/...` topics. Confirm naming before using it for data
  collection.
- It uses raw joint and raw gripper commands; caller/operator must ensure safe
  workspace, low speed, emergency stop, and correct CAN mapping.

### Dual-Arm Data Collection Without Teleoperation

Right arm:

```bash
python interface_py/lerobot_ow_right_arm.py
```

Left arm, in another terminal:

```bash
python interface_py/lerobot_ow_left_arm.py
```

Current behavior:

- `lerobot_ow_right_arm.py` reads the arm on `can0` and publishes:
  - `/right_arm/joint_states_target`
  - `/right_arm/joint_states_now`
- `lerobot_ow_left_arm.py` reads the arm on `can1` and publishes:
  - `/left_arm/joint_states_target`
  - `/left_arm/joint_states_now`
- Each script publishes 7 positions: 6 joints plus `gripper`.
- The target topic is currently the previous measured state, while the now
  topic is the current measured state.
- Each script repeatedly calls `gravity_compensation()` in a background thread.

Important notes:

- These scripts currently use infinite worker loops and minimal shutdown
  coordination.
- Some log strings and node names still say "right" in the left-arm script.
- Error messages in rare dimension-mismatch paths reference
  `self.positions_controller`, which is not defined in these two scripts.

### Dual-Arm Inference Bridge

Script:

```bash
python interface_py/lerobot_two_arm_inference.py
```

Current behavior:

- Creates two `SingleArm` instances:
  - right arm on `can0`
  - left arm on `can1`
- Publishes current robot state to `/puppet/joint`.
- Subscribes desired target state from `/master/joint`.
- Uses 14 joint names:
  - left 6 joints + `left_gripper`
  - right 6 joints + `right_gripper`

Important notes:

- The actual robot command calls are currently commented out in the control
  thread:
  - `set_joint_raw(...)`
  - `setGripperPosition_raw(...)`
- Therefore, as currently written, this script publishes state and receives
  targets but does **not** drive the arms.
- Before enabling commands, add target dimension validation. If `/master/joint`
  is missing any expected joint name, the parsed target may have fewer than 6
  joints.
- Confirm that the LeRobot bridge uses the same topic convention:
  - `/puppet/joint` for robot state
  - `/master/joint` for target commands
- Add safe shutdown coordination before using it in long-running inference.
