#!/usr/bin/env python3
"""兼容旧习惯：等价于 `python ik.py --can can1`。"""

import subprocess
import sys
from pathlib import Path


def main() -> None:
    ik_script = Path(__file__).with_name("ik.py")
    cmd = [sys.executable, str(ik_script), "--can", "can1", *sys.argv[1:]]
    raise SystemExit(subprocess.call(cmd))


if __name__ == "__main__":
    main()
