# Engineering Decisions

## 1. Mature Commercial Direction

Decision:

Design future work as a mature commercial collaborative robot controller architecture, not as temporary proof-of-concept patches.

Reason:

Temporary intermediate implementations increase validation cost and make it unclear which modules are production-intended. New work should be structured as final modules and introduced behind compatibility layers or v2 APIs.

## 2. Trajectory Baseline

Decision:

Use state-aware jerk-limited multi-axis trajectory generation as the main commercial baseline.

Reason:

Commercial robot controllers commonly emphasize:

- bounded velocity
- bounded acceleration
- bounded jerk
- smooth path blending/lookahead
- current-state replanning from position, velocity, and acceleration
- deterministic execution under controller timing constraints

Strict C3-continuous jerk or snap-limited trajectories can be useful in high-end precision or vibration-sensitive applications, but they should not be the default baseline until timing, fallback, and tuning are proven.

## 3. B-Spline, NURBS, and Blending

Decision:

Use spline/blending concepts primarily for geometric path smoothing and lookahead, then apply jerk-limited time parameterization.

Reason:

B-spline/NURBS/Bezier-style methods are appropriate for path geometry, corners, and Cartesian path smoothing. Industrial execution still needs time parameterization and limit enforcement over the path.

## 4. Old APIs Must Survive Migration

Decision:

Do not rewrite old APIs in place at the beginning. Add v2 or internal new modules first.

Reason:

The project already has working real-machine scripts. Keeping old APIs avoids disrupting current testing and allows direct comparison between old and new planners on the same VLA data.

## 5. Python Is Not the Real-Time Planner

Decision:

Python scripts are test, replay, and SDK interface layers. The real trajectory planner should be C++.

Reason:

Python loop overhead, pybind conversions, logging, plotting, and dry-run diagnostics make timing non-deterministic. A commercial controller path should keep real-time or quasi-real-time motion logic in deterministic C++ code.

## 6. Diagnostics Are Separate From Control

Decision:

Command logs, FK rows, PNG data, and debug summaries must be separated from the command path.

Reason:

The controller should not pay plotting or diagnostics cost during real-time command generation. Diagnostics should use ring buffers and explicit retrieval APIs.

## 7. Fault Codes Over String-Only Errors

Decision:

New paths should return structured fault codes plus readable messages.

Reason:

Commercial robot systems need UI display, SDK automation, logs, recovery flows, and safety decisions to agree on machine-readable error categories.

## Industry Reality Note

The architecture above is a synthesis of publicly visible industrial robot controller patterns and established motion-control libraries. Exact ABB/FANUC/UR/Yaskawa internal controller source architecture is proprietary, so this document should not claim one-to-one access to those vendors' private implementations.

Publicly visible common elements include:

- hardware abstraction, controller manager, and command/state interfaces
- jerk-limited online trajectory generation from current state
- safety functions and protective-stop style runtime supervision
- configuration-driven limits and diagnostics

