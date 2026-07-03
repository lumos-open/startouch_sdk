#!/usr/bin/env python3
"""
Process-level tuning for direct SocketCAN dual-process control.

Without USB middleware, two independent ik.py processes each run a 400Hz C++
control thread. When one process does IK (~10ms), Linux scheduling/IRQ
contention can starve the other process and trigger motor Communication lost
(50ms watchdog). USB middleware masks this by decoupling client timing from
CAN TX; ik_double.py avoids it by keeping both arms in one process.

This module pins each CAN process to a dedicated CPU and optionally raises
scheduling priority before SingleArm is constructed.
"""

from __future__ import annotations

import os
import re
import sys
from typing import Iterable, List, Optional, Set


def _parse_cpu_list(spec: str) -> Set[int]:
    cpus: Set[int] = set()
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start_s, end_s = part.split("-", 1)
            start_i = int(start_s.strip())
            end_i = int(end_s.strip())
            if start_i > end_i:
                start_i, end_i = end_i, start_i
            cpus.update(range(start_i, end_i + 1))
        else:
            cpus.add(int(part))
    return cpus


def _available_cpu_count() -> int:
    try:
        return len(os.sched_getaffinity(0))
    except AttributeError:
        return os.cpu_count() or 1


def _default_cpu_for_can(can_interface: str, cpu_count: int) -> Set[int]:
    env_key = f"STARTOUCH_CPU_AFFINITY_{can_interface.upper().replace('-', '_')}"
    override = os.environ.get(env_key, "").strip()
    if override:
        return _parse_cpu_list(override)

    global_override = os.environ.get("STARTOUCH_CPU_AFFINITY", "").strip()
    if global_override:
        return _parse_cpu_list(global_override)

    match = re.fullmatch(r"can(\d+)", can_interface)
    can_index = int(match.group(1)) if match else 0

    # RK3588 and similar: prefer big cores (4-7) when available.
    if cpu_count >= 8:
        return {min(4 + can_index, cpu_count - 1)}
    if cpu_count >= 4:
        return {can_index % cpu_count}
    return {0}


def _try_set_cpu_affinity(cpus: Iterable[int]) -> Optional[str]:
    cpu_set = sorted(set(cpus))
    if not cpu_set:
        return "skip: empty cpu set"
    try:
        os.sched_setaffinity(0, cpu_set)
        applied = sorted(os.sched_getaffinity(0))
        return f"cpu_affinity={applied}"
    except AttributeError:
        return "skip: sched_setaffinity unavailable"
    except OSError as exc:
        return f"failed: cpu_affinity ({exc})"


def _try_set_fifo_priority() -> Optional[str]:
    if os.environ.get("STARTOUCH_DISABLE_RT_SCHED", "").strip() in {"1", "true", "yes"}:
        return "skip: STARTOUCH_DISABLE_RT_SCHED=1"

    if not hasattr(os, "sched_setscheduler"):
        return "skip: sched_setscheduler unavailable"

    policy = getattr(os, "SCHED_FIFO", None)
    if policy is None:
        return "skip: SCHED_FIFO unavailable"

    try:
        max_prio = os.sched_get_priority_max(policy)
        min_prio = os.sched_get_priority_min(policy)
    except OSError as exc:
        return f"failed: sched priority range ({exc})"

    desired = int(os.environ.get("STARTOUCH_RT_PRIORITY", str(max(min_prio, max_prio - 10))))
    desired = max(min_prio, min(max_prio, desired))

    try:
        os.sched_setscheduler(0, policy, os.sched_param(desired))
        return f"sched_policy=SCHED_FIFO priority={desired}"
    except OSError as exc:
        return (
            f"failed: SCHED_FIFO ({exc}); "
            "try: sudo chrt -f 50 python3 ik.py --can <iface>"
        )


def _try_set_nice() -> Optional[str]:
    if not hasattr(os, "nice"):
        return None
    target = int(os.environ.get("STARTOUCH_NICE", "-10"))
    try:
        before = os.nice(0)
        os.nice(target - before)
        return f"nice={os.nice(0)}"
    except OSError as exc:
        return f"failed: nice ({exc})"


def apply_process_tuning(can_interface: str, *, enabled: bool = True) -> List[str]:
    if not enabled or os.environ.get("STARTOUCH_DISABLE_PROCESS_TUNING", "").strip() in {
        "1",
        "true",
        "yes",
    }:
        print(f"[ik] process tuning disabled for {can_interface}")
        return []

    cpu_count = _available_cpu_count()
    target_cpus = _default_cpu_for_can(can_interface, cpu_count)
    messages = [
        f"[ik] process tuning for {can_interface} (pid={os.getpid()}, cpus={cpu_count})",
        _try_set_cpu_affinity(target_cpus) or "",
    ]

    fifo_msg = _try_set_fifo_priority()
    if fifo_msg:
        messages.append(fifo_msg)

    nice_msg = _try_set_nice()
    if nice_msg:
        messages.append(nice_msg)

    for line in messages:
        if line:
            print(line, file=sys.stderr)
    return [line for line in messages if line]
