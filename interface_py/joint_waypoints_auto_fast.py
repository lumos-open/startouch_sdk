import argparse
import time

from startouchclass import SingleArm


HOME = [0, 0, 0, 0, 0, 0]

PATH = [
    [0, 1.54, -3, 1.29, 0, 0],
    [2.7, 1.54, -3, 1.29, 1.6, -2.7],
    [-2.7, 1.54, -3, 0, -1.6, 2.7],
    [0, 0, 0, 0, 0, 0],
    [0, 3.2, -1.5, 0, 0, 0],
    [0, -0.13, 0, -1.29, 0, 0],
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("can_interface", nargs="?", default="can0")
    parser.add_argument("--speed-percent", type=float, default=0.1)
    parser.add_argument("--time-sec", type=float, default=0.0)
    parser.add_argument("--cycles", type=int, default=1)
    args = parser.parse_args()

    arm = SingleArm(can_interface_=args.can_interface, enable_fd_=False)
    try:
        print("go home")
        arm.set_joint(HOME, tf=3)
        time.sleep(3.2)

        for i in range(max(1, args.cycles)):
            print(f"cycle {i + 1}")
            arm.move_joint_waypoints(
                PATH,
                time_sec=args.time_sec,
                speed_percent=args.speed_percent,
                ctrl_hz=400.0,
            )
            wait_s = args.time_sec + 1.0 if args.time_sec > 0 else 30.0
            time.sleep(wait_s)

        print("return home")
        arm.set_joint(HOME, tf=3)
        time.sleep(3.2)
    finally:
        arm.cleanup()


if __name__ == "__main__":
    main()
