# VLA Trajectory Replay Notes

## Current VLA Replay Scripts

Important SDK scripts:

- `startouch_sdk/interface_py/vla_replay.py`
- `startouch_sdk/interface_py/vla_replay_joint_waypoint_chunk.py`
- `startouch_sdk/interface_py/vla_replay_joint_waypoint_chunk_precheck.py`
- `startouch_sdk/interface_py/vla_replay_joint_waypoint_chunk_speed_report.py`
- legacy interface references:
  - `startouch_sdk/interface_py/pi0_rollout_single_startouch_lxh.py`
  - `startouch_sdk/interface_py/dual_touch_new_1.py`

## Current Replay Modes

`vla_replay.py` supports:

- `setjointwaypointwithgripper`: VLA pose -> IK -> joint waypoint smoothing
- `movepwithgripper`: VLA pose -> Cartesian MoveP smoothing/blending -> IK

Timing modes:

- `timesec`
- `speedpercent`

Chunk replay currently focuses on joint waypoint chunk updates, not MoveP/MoveL.

## Important Existing Behavior

`move_p_with_gripper` preserves MoveP semantics:

- pose-level path/blending first
- then IK
- not equivalent to Python pose IK followed by joint waypoints

`setjointwaypoints*` path:

- VLA pose frames are IK solved first
- joint waypoints are then planned and smoothed

With-gripper path:

- arm and gripper command targets share the same planned waypoint sample index
- arm command samples are 400Hz
- gripper hardware command/state behavior remains constrained by its lower-level 200Hz path and watchdog settings

## Known VLA Data Issue

Observed VLA chunks may have internally smooth velocity across 30 points, but acceleration and jerk can alternate sign strongly. Current PI/Wall-style model outputs should not be assumed to have learned strict acceleration or jerk continuity.

The controller must therefore smooth and constrain execution-side trajectories.

## Current Chunk Update Gaps

- suffix starts from position only, not full `q/dq/ddq`
- incoming chunk starts from point 0 instead of selecting a better entry point
- fixed `switch_delay_sec`
- linear gripper interpolation
- planned CMD samples are not execution-time CMD logs

## Required V2 Replay Behavior

For v2 tests:

- keep power-on and first-pose continuity logic
- keep Ctrl-C interruption and reset-to-zero logic
- remove artificial pauses between VLA chunks when simulating asynchronous chunk update
- support both `time_sec` and `speedpercent`
- test switching after 3, 10, and 20 points of a 30-point original chunk
- generate joint CMD vs feedback PNG with right-axis error scale
- generate TOOL XYZ+Euler CMD vs feedback PNG with right-axis error scale
- overlay original CMD-layer VLA points, not bottom-layer interpolation points

## Baseline Dry-Run Timing

Data file:

```text
/home/lumos/code/FastTouchV2/fnl/fnl/vlareplay/inferenceTraj_20260509_170209.txt
```

First group, 30 points, measured 2026-05-18:

```text
IK 30 points:          min=1.582ms avg=1.818ms max=2.140ms
Trajectory planning:   min=0.764ms avg=1.085ms max=2.371ms
IK + planning total:   min=2.359ms avg=2.907ms max=4.515ms
```

Interpretation:

- This is not final C++ OTG cost.
- It includes Python and dry-run diagnostic overhead.
- The future C++ planner should target deterministic sub-millisecond to low-millisecond planning for typical joint-space chunks.

