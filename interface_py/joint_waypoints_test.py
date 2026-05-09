import argparse
import atexit
import signal
import sys
import time

from startouchclass import SingleArm


HOME = [0.0, -0, 0.0, 0, 0.0, 0.0]


class SafeWaypointTest:
    def __init__(self, can_interface: str, speed_percent: float, time_sec: float, cycles: int):
        self.can_interface = can_interface
        self.speed_percent = speed_percent
        self.time_sec = time_sec
        self.cycles = cycles
        self.arm = None
        self.initialized = False
        self._cleaning = False
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        atexit.register(self.cleanup)

    def _signal_handler(self, signum, _frame):
        print(f"\n{self.can_interface}: received signal {signum}, stopping")
        self.cleanup()
        sys.exit(0)

    def initialize(self):
        print(f"{self.can_interface}: initializing")
        self.arm = SingleArm(can_interface_=self.can_interface, enable_fd_=False)
        self.initialized = True
        print(f"{self.can_interface}: moving home")
        self.arm.set_joint(HOME, tf=3.0)
        time.sleep(3.2)

    def cleanup(self):
        if self._cleaning or not self.initialized or self.arm is None:
            return
        self._cleaning = True
        try:
            print(f"{self.can_interface}: returning home")
            self.arm.set_joint(HOME, tf=3.0)
            time.sleep(3.2)
            self.arm.cleanup()
        except Exception as exc:
            print(f"{self.can_interface}: cleanup error: {exc}")
        finally:
            self.initialized = False

    def run(self):
        if not self.initialized:
            self.initialize()

        # Small joint-space path around HOME. Intermediate points should be passed smoothly.
        waypoints_a = [
            [0.12, 0.5, -0.08, -1, 0.10, 0.08],
            [0.52, 1.5, -0.32, -0.42, 0.18, 0.14],
            [0.12, 0.5, -0.06, -1.48, 0.08, 0.04],
            HOME,
        ]
        waypoints_b = [
            [-0.12, 0.5, -0.08, -1.5, -0.10, -0.08],
            [-0.52, 1.5, -0.12, -1, -0.18, -0.14],
            [-0.12, 0.5, -0.06, 0, -0.08, -0.04],
            HOME,
        ]

        print(
            f"{self.can_interface}: start waypoint test, cycles={self.cycles}, "
            f"speed_percent={self.speed_percent}, time_sec={self.time_sec}"
        )
        for idx in range(self.cycles):
            path = waypoints_a if idx % 2 == 0 else waypoints_b
            print(f"{self.can_interface}: cycle {idx + 1}/{self.cycles}")
            self.arm.move_joint_waypoints(
                path,
                time_sec=self.time_sec,
                speed_percent=self.speed_percent,
                ctrl_hz=400.0,
            )
            # The C++ command is asynchronous. Sleep long enough for conservative defaults;
            # when time_sec is provided, use that exact planned duration plus margin.
            wait_s = self.time_sec + 0.8 if self.time_sec > 0.0 else 8.0
            time.sleep(wait_s)
        print(f"{self.can_interface}: test done")


def parse_args():
    parser = argparse.ArgumentParser(description="Test move_joint_waypoints on one arm.")
    parser.add_argument("can_interface", nargs="?", default="can0")
    parser.add_argument("--speed-percent", type=float, default=0.1)
    parser.add_argument("--time-sec", type=float, default=0.0)
    parser.add_argument("--cycles", type=int, default=2)
    return parser.parse_args()


def main():
    args = parse_args()
    tester = SafeWaypointTest(
        can_interface=args.can_interface,
        speed_percent=args.speed_percent,
        time_sec=args.time_sec,
        cycles=max(1, args.cycles),
    )
    tester.run()


if __name__ == "__main__":
    main()
