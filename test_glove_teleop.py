#!/usr/bin/env python3
"""Small smoke test for the pure Python RealHand teleop helpers."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time

from realhand_pure_python import (
    REALHAND_MODEL_LENGTHS,
    RealForceRetarget,
    RealHandL20SdkController,
    RealMCGFrame,
    RealMCGMapper,
    RealMCGRetarget,
)
from realhand_pure_python.realforce import scan_usb_serial_ports

try:
    import yaml
except ImportError:  # pragma: no cover - JSON output still works.
    yaml = None


def make_fake_realforce_values(offset: float = 0.0) -> list[float]:
    return [0.4 + offset + i * 0.03 for i in range(21)]


def run_fake(args: argparse.Namespace) -> None:
    print("[fake] testing RealForce retarget")
    retarget = RealForceRetarget(
        left_model=args.left_model,
        right_model=args.right_model,
        calibration_path=args.calibration_path,
        load_sample_calibration=args.load_sample_calibration,
        left_calibration_side=args.left_calibration_side,
        right_calibration_side=args.right_calibration_side,
        left_thumb_abd_calibration_side=args.left_thumb_abd_calibration_side,
        right_thumb_abd_calibration_side=args.right_thumb_abd_calibration_side,
    )
    command = retarget.map_glove_positions(
        left=make_fake_realforce_values(0.0),
        right=make_fake_realforce_values(0.2),
    )
    print(f"  left positions  ({len(command.left_positions or [])}): {command.left_positions}")
    print(f"  right positions ({len(command.right_positions or [])}): {command.right_positions}")

    print("[fake] testing RealMCG mapper")
    mapper = RealMCGMapper(left_model=args.left_model, right_model=args.right_model)
    frame = RealMCGFrame(
        frame_index=1,
        left_raw=list(range(25)),
        right_raw=[24 - i for i in range(25)],
        timestamp=time.time(),
    )
    mcg_command = mapper.map_frame(frame)
    print(f"  left positions  ({len(mcg_command.left_positions)}): {mcg_command.left_positions}")
    print(f"  right positions ({len(mcg_command.right_positions)}): {mcg_command.right_positions}")
    print("[ok] fake-data smoke test passed")
    send_to_l20_sdk(args, command.left_positions, command.right_positions)


def run_scan() -> None:
    ports = scan_usb_serial_ports()
    if not ports:
        print("No USB serial ports found.")
        return
    print("USB serial ports:")
    for port in ports:
        print(f"  {port}")


def run_realforce_calibration(args: argparse.Namespace) -> None:
    _enable_realtime_output()
    _calibration_print("[calibration] starting RealForce calibration capture")
    missing_ports = [port for port in (args.left_port, args.right_port) if port and not Path(port).exists()]
    if missing_ports:
        _calibration_print(f"[calibration] missing port(s): {missing_ports}")
        return

    baudrate = None if args.baudrate == 0 else args.baudrate
    logger = (lambda msg: _calibration_print(f"[realforce] {msg}")) if args.serial_debug else None
    retarget = RealForceRetarget(
        left_model=args.left_model,
        right_model=args.right_model,
        left_port=args.left_port,
        right_port=args.right_port,
        baudrate=baudrate,
        serial_query_hz=args.serial_query_hz,
        load_sample_calibration=False,
        logger=logger,
    )
    retarget.start()
    try:
        _calibration_print(
            "[calibration] opened",
            {
                "left": retarget.reader.left_opened,
                "right": retarget.reader.right_opened,
                "baudrate": "auto" if baudrate is None else baudrate,
            },
        )
        if retarget.reader.left_opened is None and retarget.reader.right_opened is None:
            _calibration_print("[calibration] no RealForce glove opened; use --serial-debug and check the ports.")
            return

        _wait_for_calibration_frames(retarget.reader, timeout_s=3.0)
        data = {"timestamp": datetime.now().isoformat()}
        poses = [
            ("original", "jointangleoriginal", "五指自然张开、手指放平"),
            ("opose", "jointangleopose", "拇指和食指做 O 型，其余手指自然"),
            ("fist", "jointanglefist", "握紧拳头"),
        ]
        for pose_name, key_prefix, prompt in poses:
            _prepare_calibration_pose(pose_name, prompt, args)
            time.sleep(args.calibration_settle_seconds)
            _calibration_print(
                f"[calibration] {pose_name}: 开始采集 {args.calibration_sample_seconds:.1f}s，"
                "请保持姿态不动..."
            )
            left_samples, right_samples = _capture_calibration_samples(
                retarget.reader,
                duration_s=args.calibration_sample_seconds,
                sample_hz=args.serial_query_hz,
            )
            left_avg = _weighted_average(left_samples)
            right_avg = _weighted_average(right_samples)
            if left_avg is not None:
                data[f"{key_prefix}_l"] = left_avg
                _calibration_print(f"[calibration] left {pose_name}: {_sample_summary(left_samples)}")
            if right_avg is not None:
                data[f"{key_prefix}_r"] = right_avg
                _calibration_print(f"[calibration] right {pose_name}: {_sample_summary(right_samples)}")
            if left_avg is None and right_avg is None:
                raise RuntimeError(f"no valid samples captured for {pose_name}")

        output_path = Path(args.calibration_output)
        _write_calibration_file(output_path, data)
        _calibration_print(f"\n[calibration] saved {output_path}")
    finally:
        retarget.stop()


def _enable_realtime_output() -> None:
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if callable(reconfigure):
            reconfigure(line_buffering=True, write_through=True)


def _calibration_print(*args, **kwargs) -> None:
    kwargs.setdefault("flush", True)
    print(*args, **kwargs)


def _prepare_calibration_pose(pose_name: str, prompt: str, args: argparse.Namespace) -> None:
    title = f"[calibration] {pose_name}"
    message = f"{title}: 请摆出姿态：{prompt}"
    _calibration_print("\n" + "=" * 72)
    _calibration_print(message)
    _calibration_print("=" * 72)
    if args.calibration_interactive:
        try:
            input(f"{title}: 摆好并保持不动后按 Enter 开始采集...")
            return
        except EOFError:
            _calibration_print(f"{title}: stdin 不可用，改用倒计时自动采集。")

    seconds = max(0.0, float(args.calibration_ready_seconds))
    if seconds <= 0:
        return
    deadline = time.monotonic() + seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        _calibration_print(f"{title}: {remaining:.1f}s 后开始采集。现在摆好：{prompt}")
        time.sleep(min(1.0, remaining))


def _wait_for_calibration_frames(reader, *, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        left_snapshot, right_snapshot = reader.snapshots()
        if is_valid_realforce_snapshot(left_snapshot) or is_valid_realforce_snapshot(right_snapshot):
            return
        time.sleep(0.05)


def _capture_calibration_samples(reader, *, duration_s: float, sample_hz: float) -> tuple[list[list[float]], list[list[float]]]:
    left_samples: list[list[float]] = []
    right_samples: list[list[float]] = []
    period = 1.0 / sample_hz if sample_hz > 0 else 1.0 / 60.0
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        left_snapshot, right_snapshot = reader.snapshots()
        if is_valid_realforce_snapshot(left_snapshot):
            left_samples.append(list(left_snapshot.positions))
        if is_valid_realforce_snapshot(right_snapshot):
            right_samples.append(list(right_snapshot.positions))
        time.sleep(min(period, 0.05))
    return left_samples, right_samples


def _weighted_average(samples: list[list[float]]) -> list[float] | None:
    if not samples:
        return None
    length = len(samples[0])
    usable = [sample for sample in samples if len(sample) == length]
    if not usable:
        return None
    center = (len(usable) - 1) / 2.0
    weights = [1.0 if center == 0 else 1.0 - abs(index - center) / (center + 1.0) for index in range(len(usable))]
    total_weight = sum(weights)
    return [
        sum(sample[index] * weight for sample, weight in zip(usable, weights)) / total_weight
        for index in range(length)
    ]


def _sample_summary(samples: list[list[float]]) -> dict:
    if not samples:
        return {"samples": 0}
    first = samples[0]
    last = samples[-1]
    length = min(len(first), len(last))
    drift = sum(abs(last[index] - first[index]) for index in range(length)) / max(length, 1)
    return {"samples": len(samples), "drift": round(drift, 5)}


def _write_calibration_file(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix.lower() == ".json":
        path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        return
    if yaml is None:
        raise ImportError("PyYAML is required to write YAML calibration files; use a .json output path instead.")
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")


def run_serial(args: argparse.Namespace) -> None:
    print("[serial] starting RealForce reader")
    print(f"[retarget] left_model={args.left_model} right_model={args.right_model}")
    print(
        "[retarget] calibration",
        args.calibration_path if args.calibration_path else ("sample" if args.load_sample_calibration else "none"),
        {
            "left_side": args.left_calibration_side,
            "right_side": args.right_calibration_side,
            "left_thumb_abd_side": args.left_thumb_abd_calibration_side,
            "right_thumb_abd_side": args.right_thumb_abd_calibration_side,
        },
    )
    if args.sdk_send:
        print(
            "[sdk] using RealHand SDK",
            {
                "left": resolve_sdk_model(args.left_sdk_model, args.left_model),
                "right": resolve_sdk_model(args.right_sdk_model, args.right_model),
            },
        )
    missing_ports = [port for port in (args.left_port, args.right_port) if port and not Path(port).exists()]
    if missing_ports:
        print(f"[serial] missing port(s): {missing_ports}")
        print("[serial] run `python test_glove_teleop.py --scan` after plugging in the RealForce USB receiver.")
        return

    controller = None
    baudrate = None if args.baudrate == 0 else args.baudrate
    logger = (lambda msg: print(f"[realforce] {msg}")) if args.serial_debug else None
    retarget = RealForceRetarget(
        left_model=args.left_model,
        right_model=args.right_model,
        left_port=args.left_port,
        right_port=args.right_port,
        baudrate=baudrate,
        serial_query_hz=args.serial_query_hz,
        calibration_path=args.calibration_path,
        load_sample_calibration=args.load_sample_calibration,
        left_calibration_side=args.left_calibration_side,
        right_calibration_side=args.right_calibration_side,
        left_thumb_abd_calibration_side=args.left_thumb_abd_calibration_side,
        right_thumb_abd_calibration_side=args.right_thumb_abd_calibration_side,
        logger=logger,
    )
    retarget.start()
    print(
        "[serial] opened",
        {
            "left": retarget.reader.left_opened,
            "right": retarget.reader.right_opened,
            "baudrate": "auto" if baudrate is None else baudrate,
        },
    )
    print(
        "[serial] rates",
        {
            "serial_query_hz": args.serial_query_hz,
            "send_hz": args.send_hz,
            "print_hz": args.print_hz,
        },
    )
    if retarget.reader.left_opened is None and retarget.reader.right_opened is None:
        print("[serial] no RealForce glove opened; use --serial-debug and verify the /dev/ttyUSB* ports are not busy.")
    try:
        deadline = time.monotonic() + args.seconds
        send_period = 1.0 / args.send_hz if args.send_hz > 0 else max(args.interval, 0.001)
        print_period = 1.0 / args.print_hz if args.print_hz > 0 else None
        next_send_time = time.monotonic()
        next_print_time = next_send_time
        prev_left_positions = None
        prev_right_positions = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_send_time:
                time.sleep(min(next_send_time - now, 0.005))
                continue
            next_send_time += send_period
            if next_send_time < now:
                next_send_time = now + send_period

            command = retarget.poll()
            left_snapshot, right_snapshot = retarget.reader.snapshots()
            left_ok = is_valid_realforce_snapshot(left_snapshot)
            right_ok = is_valid_realforce_snapshot(right_snapshot)
            should_print = print_period is not None and now >= next_print_time
            if should_print:
                print(
                    "left",
                    snapshot_summary(
                        left_snapshot,
                        left_ok,
                        command.left_positions,
                        prev_left_positions,
                        args,
                    ),
                )
                print(
                    "right",
                    snapshot_summary(
                        right_snapshot,
                        right_ok,
                        command.right_positions,
                        prev_right_positions,
                        args,
                    ),
                )
                prev_left_positions = left_snapshot.positions
                prev_right_positions = right_snapshot.positions
                next_print_time = now + print_period
            if args.sdk_send and not (left_ok or right_ok):
                if should_print:
                    print("[serial] no valid RealForce frame yet; not sending CAN command")
                time.sleep(args.interval)
                continue
            if args.sdk_send and controller is None:
                controller = make_l20_sdk_controller(args)
            send_to_l20_sdk(
                args,
                command.left_positions if left_ok else None,
                command.right_positions if right_ok else None,
                controller=controller,
            )
    finally:
        retarget.stop()
        if controller is not None:
            controller.close()


def is_valid_realforce_snapshot(snapshot) -> bool:
    if not snapshot.connected:
        return False
    if snapshot.version is None:
        return False
    if not snapshot.positions:
        return False
    return any(abs(value) > 1e-9 for value in snapshot.positions)


def snapshot_summary(snapshot, valid: bool, command_positions, previous_positions, args: argparse.Namespace) -> dict:
    now = time.time()
    positions = snapshot.positions
    if previous_positions and len(previous_positions) == len(positions):
        raw_delta_max = max(abs(a - b) for a, b in zip(positions, previous_positions))
    else:
        raw_delta_max = None

    raw_values = positions if args.raw_all else positions[:5]
    command_values = command_positions or []
    summary = {
        "connected": snapshot.connected,
        "valid": valid,
        "side": snapshot.side,
        "version": snapshot.version,
        "frames": snapshot.frame_count,
        "pos_frames": snapshot.position_frame_count,
        "rx_age_s": round(now - snapshot.last_rx_time, 3) if snapshot.last_rx_time else None,
        "pos_age_s": round(now - snapshot.last_position_time, 3) if snapshot.last_position_time else None,
        "raw_delta_max": round(raw_delta_max, args.raw_precision) if raw_delta_max is not None else None,
        "raw": round_list(raw_values, args.raw_precision),
        "cmd": command_values if args.cmd_all else command_values[:5],
    }
    return summary


def round_list(values, precision: int) -> list[float]:
    return [round(float(value), precision) for value in values]


def run_mcg(args: argparse.Namespace) -> None:
    mode = "serial" if args.mcg_serial_port else "UDP"
    print(f"[mcg] starting RealMCG {mode} reader")
    print(f"[retarget] left_model={args.left_model} right_model={args.right_model}")
    if args.sdk_send:
        print(
            "[sdk] using",
            {
                "left": resolve_sdk_model(args.left_sdk_model, args.left_model),
                "right": resolve_sdk_model(args.right_sdk_model, args.right_model),
            },
        )
    controller = make_l20_sdk_controller(args)
    bind_address = None
    if args.mcg_bind_port is not None:
        bind_address = (args.mcg_bind_host, args.mcg_bind_port)
        print(f"  local bind: {bind_address[0]}:{bind_address[1]}")
    if args.mcg_serial_port:
        print(f"  serial: {args.mcg_serial_port} @ {args.mcg_serial_baudrate}")
    else:
        print(f"  target: {args.mcg_host}:{args.mcg_port}")

    retarget = RealMCGRetarget(
        host=args.mcg_host,
        port=args.mcg_port,
        left_model=args.left_model,
        right_model=args.right_model,
        bind_address=bind_address,
        serial_port=args.mcg_serial_port,
        serial_baudrate=args.mcg_serial_baudrate,
        logger=lambda msg: print(f"  {msg}"),
    )
    retarget.start()
    try:
        deadline = time.monotonic() + args.seconds
        poll_period = 1.0 / args.send_hz if args.send_hz > 0 else max(args.interval, 0.001)
        print_period = 1.0 / args.print_hz if args.print_hz > 0 else None
        next_poll_time = time.monotonic()
        next_print_time = next_poll_time
        last_frame = -1
        last_printed_frame = -1
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_poll_time:
                time.sleep(min(next_poll_time - now, 0.005))
                continue
            next_poll_time += poll_period
            if next_poll_time < now:
                next_poll_time = now + poll_period

            command = retarget.latest_command
            if command is None:
                if print_period is not None and now >= next_print_time:
                    print("  waiting for MCG UDP JSON frames...")
                    next_print_time = now + print_period
            elif command.frame_index != last_frame:
                last_frame = command.frame_index
                if (
                    print_period is not None
                    and now >= next_print_time
                    and command.frame_index != last_printed_frame
                ):
                    last_printed_frame = command.frame_index
                    print(
                        f"  frame {command.frame_index}: "
                        f"left_raw={command.left_raw[:5]} left_cmd={command.left_positions[:6]} "
                        f"right_raw={command.right_raw[:5]} right_cmd={command.right_positions[:6]}"
                    )
                    next_print_time = now + print_period
                send_to_l20_sdk(args, command.left_positions, command.right_positions, controller=controller)
    finally:
        retarget.stop()
        if controller is not None:
            controller.close()


def parse_int_values(text: str, *, expected: int | None = None, name: str) -> list[int]:
    values = [int(float(part)) for part in text.replace(",", " ").split()]
    if expected is not None and len(values) != expected:
        raise ValueError(f"{name} must contain {expected} values")
    return values


def make_l20_sdk_controller(args: argparse.Namespace) -> RealHandL20SdkController | None:
    if not args.sdk_send:
        return None
    left_sdk_model = resolve_sdk_model(args.left_sdk_model, args.left_model)
    right_sdk_model = resolve_sdk_model(args.right_sdk_model, args.right_model)
    controller = RealHandL20SdkController(
        left_can=args.left_can,
        right_can=args.right_can,
        left_model=left_sdk_model,
        right_model=right_sdk_model,
    )
    speed = parse_int_values(args.sdk_speed, name="--sdk-speed")
    controller.set_speed(left_speed=speed, right_speed=speed)
    print(f"[sdk] connected hands: {controller.status}")
    if args.open_before_control:
        status = controller.status
        left_open = open_pose_for_model(left_sdk_model) if status.left_connected else None
        right_open = open_pose_for_model(right_sdk_model) if status.right_connected else None
        print(f"[sdk] opening hands before teleop for {args.open_hold_s:.2f}s")
        controller.move(left_pose=left_open, right_pose=right_open)
        time.sleep(args.open_hold_s)
    return controller


def resolve_sdk_model(sdk_model: str, retarget_model: str) -> str:
    model = sdk_model.lower()
    if model == "auto":
        return "l6" if retarget_model.lower() == "l6" else "l20"
    if model == "g20":
        return "l20"
    return model


def open_pose_for_model(model: str) -> list[int]:
    length = REALHAND_MODEL_LENGTHS.get(model.lower(), 20)
    if length == 6:
        return [255] * 6
    return [255] * 20


def send_to_l20_sdk(
    args: argparse.Namespace,
    left_positions: list[int] | None,
    right_positions: list[int] | None,
    *,
    controller: RealHandL20SdkController | None = None,
) -> None:
    if not args.sdk_send:
        return
    own_controller = controller is None
    controller = controller or make_l20_sdk_controller(args)
    try:
        if left_positions is not None and len(left_positions) not in (6, 20):
            raise ValueError(f"left command must have 6 L6 values or 20 L20/G20 values, got {len(left_positions)}")
        if right_positions is not None and len(right_positions) not in (6, 20):
            raise ValueError(f"right command must have 6 L6 values or 20 L20/G20 values, got {len(right_positions)}")
        controller.move(left_pose=left_positions, right_pose=right_positions)
    finally:
        if own_controller and controller is not None:
            controller.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-model", default="g20", help="retarget model, for example l20 or g20")
    parser.add_argument("--right-model", default="g20", help="retarget model, for example l20 or g20")
    parser.add_argument("--scan", action="store_true", help="list USB serial ports and exit")
    parser.add_argument("--calibrate-realforce", action="store_true", help="capture open/O/fist RealForce calibration poses")
    parser.add_argument("--serial", action="store_true", help="read RealForce serial hardware")
    parser.add_argument("--mcg", action="store_true", help="read RealMCG UDP broadcast or USB serial receiver")
    parser.add_argument("--left-port", help="left glove serial port, for example /dev/ttyUSB0")
    parser.add_argument("--right-port", help="right glove serial port, for example /dev/ttyUSB1")
    parser.add_argument("--baudrate", type=int, default=0, help="RealForce baudrate; 0 means auto-scan")
    parser.add_argument("--calibration-path", help="RealForce calibration YAML/JSON path")
    parser.add_argument(
        "--calibration-output",
        default="realhand_pure_python/config/calibration_user.yml",
        help="where --calibrate-realforce writes the captured calibration file",
    )
    parser.add_argument("--calibration-sample-seconds", type=float, default=3.0, help="seconds sampled for each calibration pose")
    parser.add_argument("--calibration-settle-seconds", type=float, default=0.5, help="settle time after pressing Enter for each pose")
    parser.add_argument("--calibration-ready-seconds", type=float, default=5.0, help="countdown seconds before each calibration pose capture")
    parser.add_argument("--calibration-interactive", action="store_true", help="wait for Enter before each calibration pose when stdin is interactive")
    parser.add_argument(
        "--load-sample-calibration",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="load bundled sample calibration when --calibration-path is not set",
    )
    parser.add_argument(
        "--left-calibration-side",
        default="left",
        choices=["left", "right"],
        help="which calibration side to use for left glove raw values",
    )
    parser.add_argument(
        "--right-calibration-side",
        default="right",
        choices=["left", "right"],
        help="which calibration side to use for right glove raw values",
    )
    parser.add_argument(
        "--left-thumb-abd-calibration-side",
        choices=["left", "right"],
        help="optional calibration side override for the left thumb abduction channel",
    )
    parser.add_argument(
        "--right-thumb-abd-calibration-side",
        choices=["left", "right"],
        help="optional calibration side override for the right thumb abduction channel",
    )
    parser.add_argument("--serial-debug", action="store_true", help="print RealForce serial open/read diagnostics")
    parser.add_argument("--serial-query-hz", type=float, default=60.0, help="RealForce serial position query rate")
    parser.add_argument("--raw-all", action="store_true", help="print all 21 RealForce raw values instead of the first 5")
    parser.add_argument("--raw-precision", type=int, default=4, help="decimal precision for RealForce raw values")
    parser.add_argument("--cmd-all", action="store_true", help="print all mapped command values instead of the first 5")
    parser.add_argument("--mcg-host", default="127.0.0.1", help="MCG UDP target host")
    parser.add_argument("--mcg-port", type=int, default=9011, help="MCG UDP target port; manual default is 9011")
    parser.add_argument("--mcg-bind-host", default="0.0.0.0", help="local host for receiving pushed MCG UDP frames")
    parser.add_argument("--mcg-bind-port", type=int, help="local port for receiving pushed MCG UDP frames")
    parser.add_argument("--mcg-serial-port", help="RealMCG USB serial receiver port, for example /dev/ttyUSB2")
    parser.add_argument("--mcg-serial-baudrate", type=int, default=115200, help="RealMCG serial baudrate")
    parser.add_argument("--sdk-send", action="store_true", help="send mapped L20/G20-compatible commands through the official SDK")
    parser.add_argument("--left-can", default="can0", help="left L20/G20 CAN interface")
    parser.add_argument("--right-can", default="can1", help="right L20/G20 CAN interface")
    parser.add_argument("--left-sdk-model", default="auto", choices=["auto", "l6", "l20", "g20"], help="left SDK hardware model; auto follows --left-model")
    parser.add_argument("--right-sdk-model", default="auto", choices=["auto", "l6", "l20", "g20"], help="right SDK hardware model; auto follows --right-model")
    parser.add_argument("--sdk-speed", default="100,100,100,100,100", help="5 grouped speeds or 6 L6 speed values")
    parser.add_argument(
        "--open-before-control",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="send an open pose once before teleop control starts",
    )
    parser.add_argument("--open-hold-s", type=float, default=1.0, help="seconds to hold the pre-control open pose")
    parser.add_argument("--send-hz", type=float, default=50.0, help="serial retarget/CAN send rate; set lower if CAN is overloaded")
    parser.add_argument("--print-hz", type=float, default=5.0, help="serial status print rate; use 0 to disable printing")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--interval", type=float, default=0.5, help="legacy sleep interval and MCG polling interval")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.scan:
        run_scan()
    elif args.calibrate_realforce:
        run_realforce_calibration(args)
    elif args.serial:
        run_serial(args)
    elif args.mcg:
        run_mcg(args)
    else:
        run_fake(args)


if __name__ == "__main__":
    main()
