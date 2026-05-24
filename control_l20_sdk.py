#!/usr/bin/env python3
"""Control L20/G20-compatible hands through the official RealHand Python SDK."""

from __future__ import annotations

import argparse
import time

from realhand_pure_python import DEFAULT_L20_OPEN_POSE, RealForceRetarget, RealHandL20SdkController


def parse_values(text: str, *, expected: int, name: str) -> list[int]:
    parts = text.replace(",", " ").split()
    values = [int(float(part)) for part in parts]
    if len(values) != expected:
        raise argparse.ArgumentTypeError(f"{name} must contain {expected} values")
    return values


def build_pose(args: argparse.Namespace) -> tuple[list[int] | None, list[int] | None]:
    if args.action == "open":
        pose = [args.open_value] * 20
        return pose, pose

    if args.action == "fake-glove":
        retarget = RealForceRetarget(left_model="l20", right_model="l20")
        command = retarget.map_glove_positions(
            left=[0.4 + i * 0.03 for i in range(21)],
            right=[0.6 + i * 0.03 for i in range(21)],
        )
        return command.left_positions, command.right_positions

    if args.action == "pose":
        left = parse_values(args.left_pose, expected=20, name="--left-pose") if args.left_pose else None
        right = parse_values(args.right_pose, expected=20, name="--right-pose") if args.right_pose else None
        if left is None and right is None:
            raise SystemExit("action=pose requires --left-pose and/or --right-pose")
        return left, right

    return None, None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-can", default="can0")
    parser.add_argument("--right-can", default="can1")
    parser.add_argument("--action", choices=["check", "open", "fake-glove", "pose"], default="check")
    parser.add_argument("--left-pose", help="20 L20/G20 values, separated by spaces or commas")
    parser.add_argument("--right-pose", help="20 L20/G20 values, separated by spaces or commas")
    parser.add_argument("--open-value", type=int, default=250)
    parser.add_argument("--speed", default="100,100,100,100,100", help="5 speed values for L20/G20")
    parser.add_argument("--send", action="store_true", help="actually connect to CAN and send commands")
    parser.add_argument("--read-state", action="store_true", help="read state after connecting or moving")
    parser.add_argument("--sleep", type=float, default=0.2)
    args = parser.parse_args()

    left_pose, right_pose = build_pose(args)
    speed = parse_values(args.speed, expected=5, name="--speed")

    print(f"left_can={args.left_can} right_can={args.right_can} action={args.action}")
    if left_pose is not None:
        print(f"left_pose ({len(left_pose)}): {left_pose}")
    if right_pose is not None:
        print(f"right_pose ({len(right_pose)}): {right_pose}")
    print(f"speed: {speed}")

    if not args.send:
        print("[dry-run] add --send to connect to CAN and control the L20/G20 hands")
        return

    controller = RealHandL20SdkController(left_can=args.left_can, right_can=args.right_can)
    try:
        print(f"connected: {controller.status}")
        controller.set_speed(left_speed=speed, right_speed=speed)
        if left_pose is not None or right_pose is not None:
            controller.move(left_pose=left_pose, right_pose=right_pose)
            time.sleep(args.sleep)
        if args.read_state or args.action == "check":
            print(f"state: {controller.get_state()}")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
