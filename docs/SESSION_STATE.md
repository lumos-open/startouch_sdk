# Session State

Last updated: 2026-05-18

## Recovery Command

If the chat, network, or Codex session disconnects, resume with:

```text
查看 docs/SESSION_STATE.md 和 docs/DEVELOPMENT_PLAN.md，继续当前进展。
```

For VLA replay-specific work, also ask to read:

```text
查看 docs/VLA_TRAJECTORY_REPLAY_NOTES.md，继续。
```

## Repository Layout

- `startouchlib`: bottom-layer C++ source and robot controller implementation.
- `startouch_sdk`: SDK pybind, Python wrappers, and VLA replay/interface scripts.
- Shared planning docs live in `/home/lumos/code/FastTouchV2/fnl/fnl/docs`.

## Current Branches

- `startouchlib`: `charlie`
- `startouch_sdk`: `charlie`

Future commits and pushes should go directly to the `charlie` branch in both repos when requested.
Git identity should use company account `charlie@lumosbot.tech`.

## Latest Known Pushed Commits

- `startouchlib`: `0c1c23252808be1614e9ddac07cf2bd9c21a0d59`
  - Message: `新增关节路点chunk更新与离线规划`
- `startouch_sdk`: `f7c3aca8382fde72543a3bf7660b65278a8a31e4`
  - Message: `新增VLA回放chunk预检与同步接口`

Historical SDK pipeline failure commit `ec9eb79d21bf7d40a1f698246222757babcd31d8` is not the latest commit and was treated as not blocking unless future CI still fails.

## Current Working Tree Notes

At the time this file was created:

- `startouchlib` was clean before docs were added.
- `startouch_sdk` had uncommitted replay-related changes:
  - Modified: `interface_py/vla_replay_joint_waypoint_chunk.py`
  - Untracked: `interface_py/vla_replay_joint_waypoint_chunk_speed_report.py`
  - Untracked runtime PNG artifacts:
    - `vla_replay_joint_cmd_vs_feedback.png`
    - `vla_replay_tool_cmd_vs_feedback.png`

Do not revert these without explicit user approval.

## Implemented Technical Context

Existing recent work includes:

- `move_joint_waypoints_with_gripper`
- `move_p_with_gripper`
- `update_joint_waypoint_chunk`
- `update_joint_waypoint_chunk_with_gripper`
- `plan_joint_waypoints_with_gripper`
- `get_last_waypoint_command_samples`
- dry-run construction support for offline planning checks
- `vla_replay.py` replacing the previous `vla_replay_move_p_with_gripper.py`
- chunk replay and precheck scripts for VLA action chunk update simulation

Current chunk update behavior:

- Keeps a short prefix from the currently active trajectory.
- Plans a suffix from the retained prefix end point to the new chunk.
- Replaces future samples with the combined prefix and suffix.
- With-gripper path uses the same sample index/time axis as the arm command samples.

Known technical gaps:

- Chunk suffix currently inherits position but not full `q/dq/ddq` boundary state.
- New chunk currently starts from `new_chunk[0]` instead of selecting a better entry point.
- `switch_delay_sec` is a fixed input, not derived from measured planning time.
- `time_sec` mode currently distributes time by segment count, not by path length or source timestamps.
- Gripper samples are linear interpolation, not jerk-limited.
- `get_last_waypoint_command_samples()` is planned CMD data, not an execution-time 400Hz CMD log.

## Latest Planning-Time Measurement

Using `vla_replay.py` first group of 30 points from:

```text
/home/lumos/code/FastTouchV2/fnl/fnl/vlareplay/inferenceTraj_20260509_170209.txt
```

Observed dry-run timing on 2026-05-18:

```text
IK 30 points:          min=1.582ms avg=1.818ms max=2.140ms
Trajectory planning:   min=0.764ms avg=1.085ms max=2.371ms
IK + planning total:   min=2.359ms avg=2.907ms max=4.515ms
```

This includes Python loop, pybind conversion, KDL IK, planning, and dry-run return data. It should not be treated as the final C++ real-time OTG cost.

## Immediate Discussion State

The user wants the next architecture to follow a mature commercial collaborative robot style, not temporary minimum viable implementations.

Current intended direction:

- Build a C++ state-aware jerk-limited multi-axis trajectory generator.
- Keep real-time command generation free of Python, plotting, FK diagnostics, and direct logging.
- Separate motion core, command runtime, safety manager, fault codes, config schema, diagnostics, and API layer.
- Preserve old APIs during migration and introduce v2 paths for validation before switching defaults.

## High-Frequency Recovery Files

In addition to this state file:

- `docs/SESSION_LOG.md` records key user questions, conclusions, and work checkpoints between commits.
- `docs/NEXT_ACTIONS.md` records the current short-term task state.

The assistant should update these files proactively during substantial discussion, before coding, after coding, after tests, and before commits.
