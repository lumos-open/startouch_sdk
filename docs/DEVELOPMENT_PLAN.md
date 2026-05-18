# Development Plan

## Goal

Move `startouchlib` and `startouch_sdk` toward a mature commercial collaborative robot control architecture while preserving current working interfaces during migration.

The target is not a temporary minimum version. New work should be designed as final architecture and introduced behind compatible APIs or explicit v2 APIs until validated.

## Architecture Target

### 1. Motion Core

Create a C++ state-aware jerk-limited multi-axis trajectory generator.

Responsibilities:

- input current state `q/dq/ddq`
- support joint waypoints and gripper waypoints on one time axis
- support `speed_percent` and `time_sec`
- support velocity, acceleration, and jerk limits
- support multi-waypoint lookahead and blending
- support chunk replacement from an active trajectory state
- output deterministic command samples and a compact planning summary

This module must not depend on Python, plotting, FK diagnostics, or console logging.

### 2. Command Runtime

The control loop should consume prepared command buffers or deterministic online trajectory results.

Responsibilities:

- fixed-rate command execution
- active trajectory switching
- execution-time command logging to a ring buffer
- tracking error handling
- motion state reporting

The control loop should avoid dynamic allocation, direct `cerr` logging, expensive IK/FK, and Python involvement.

### 3. Safety Manager

Centralize safety checks and runtime safety events.

Scope:

- joint limits
- velocity, acceleration, and jerk planning limits
- tracking error
- torque limits
- CAN or motor communication timeout
- gripper watchdog
- singularity and near-singularity protection
- emergency stop and protective stop paths

Safety should return structured events with severity and fault codes.

### 4. Fault Code System

Replace scattered string-only errors in new paths with structured fault results.

Example codes:

- `MOTION_TIME_TOO_SHORT`
- `MOTION_JOINT_LIMIT`
- `MOTION_VELOCITY_LIMIT`
- `MOTION_ACCELERATION_LIMIT`
- `MOTION_JERK_LIMIT`
- `MOTION_SINGULARITY`
- `MOTION_IK_FAILED`
- `CONTROL_TRACKING_ERROR`
- `CONTROL_CAN_TIMEOUT`
- `GRIPPER_TIMEOUT`
- `CONFIG_INVALID`

The Python SDK should expose both readable messages and machine-readable codes.

### 5. Config System

Move toward validated, versioned config with clear ownership:

- `robot_model.yaml`: tool, payload, mounting, URDF-related model parameters
- `motion_limits.yaml`: joint and gripper velocity, acceleration, jerk, default profiles
- `safety.yaml`: safety thresholds and stop policies
- `controller.yaml`: control frequency, state frequency, watchdog, command logging
- `sdk_runtime.yaml`: Python script defaults and replay-only settings

Startup must validate config and fail clearly on invalid values.

### 6. Diagnostics

Separate planning, execution, feedback, and fault diagnostics.

Interfaces to add over time:

- `get_last_plan_summary()`
- `get_command_log()`
- `get_feedback_log()`
- `get_fault_events()`
- `clear_faults()`

PNG generation and replay analysis should read diagnostics data, not change controller behavior.

### 7. API Layer

Keep existing APIs during migration:

- `move_joint_waypoints`
- `move_joint_waypoints_with_gripper`
- `update_joint_waypoint_chunk`
- `update_joint_waypoint_chunk_with_gripper`

Add v2 APIs for validation before making them defaults:

- `move_joint_waypoints_v2`
- `move_joint_waypoints_with_gripper_v2`
- `update_joint_waypoint_chunk_v2`
- `update_joint_waypoint_chunk_with_gripper_v2`

Old APIs can later forward to v2 after validation.

## Migration Plan

### Phase 1: Design and Dry-Run Core

Deliver:

- new motion types and planning result structs
- new state-aware jerk-limited trajectory core
- structured fault code types
- dry-run C++ and Python bindings
- VLA replay dry-run comparison against existing data
- benchmark report for planning time, max velocity, max acceleration, max jerk, and boundary continuity

No default real-machine execution path changes in this phase.

### Phase 2: Chunk v2 Execution

Deliver:

- v2 chunk update execution path
- inherit active trajectory `q/dq/ddq` at switch point
- dynamic switch delay based on planning-time budget
- entry-point selection for incoming chunks
- gripper on the same state-aware trajectory time axis
- execution-time 400Hz command log

### Phase 3: Safety, Config, and Diagnostics Unification

Deliver:

- safety manager integration
- structured fault reporting through C++ and Python
- config schema validation
- diagnostics ring buffers
- replay scripts using structured logs and fault summaries

### Phase 4: Default Switch and Deprecation

Deliver:

- existing APIs internally forward to v2
- old planner marked deprecated
- CI/dry-run tests cover old and new external behavior
- remove old implementation only after a stable validation period

## Validation Matrix

Use fixed VLA replay data and synthetic edge cases:

- 30-point normal playback
- 3/10/20-point chunk replacement
- `time_sec`
- `speed_percent`
- gripper synchronized playback
- too-short time
- large boundary jump
- joint limit
- velocity/acceleration/jerk limit
- IK failure
- singularity or near-singularity
- emergency stop or Ctrl-C recovery path

Metrics:

- planning time min/avg/max
- worst-case allocation/logging on planning path
- sample duration
- max velocity/acceleration/jerk per joint
- boundary position/velocity/acceleration discontinuity
- gripper timing alignment
- feedback alignment error after interpolation

