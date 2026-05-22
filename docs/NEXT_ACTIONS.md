# Next Actions

Last updated: 2026-05-22 CST

## Current Focus

Current implementation has passed the user's local hardware test. Prepare separate traceable commits and push both `startouchlib` and `startouch_sdk` directly to branch `charlie`.

## Explicitly Not Current Tasks

- Do not continue the one-off "check potential code bugs" review. The user clarified it was only a speed test.

## Pending Decisions

- OTG/state-aware jerk-limited planner remains a later refactor and is not part of the current tested push.
- Keep current APIs stable for user testing before larger industrial motion-core restructuring.

## Immediate Next Step

- Commit current changes as separate, reviewable commits.
- Do not commit generated PNG replay artifacts unless explicitly requested.
- Push both repositories to `origin/charlie` using `charlie@lumosbot.tech`.
