# RealHand Pure Python

Standalone Python helpers for RealForce and RealMCG teleoperation data.

For the validated RealForce glove to dual L6 CAN teleoperation workflow,
including user calibration and run commands, see
[`REALFORCE_L6_TELEOP_README.md`](../REALFORCE_L6_TELEOP_README.md).

This folder is intentionally outside `ros1/` and `ros2/`. It does not import
`rclpy`, `rospy`, or ROS message types.

## What It Provides

- `realforce.py`: reads RealForce USB serial data through `pyserial`.
- `realforce_retarget.py`: maps 21-value RealForce glove arrays to RealHand
  motor command lists without ROS.
- `realmcg.py`: receives RealMCG UDP JSON frames and maps them to command lists.
- `realhand_core.py`: parses local URDF joint limits and applies the RealHand
  command scaling from `config/hand_config.yml`.
- All outputs are plain Python lists, so another hardware-control repo can send
  them directly to the hand.

## RealForce

Quick smoke test from the repo root:

```bash
conda activate gloveTeleop
python test_glove_teleop.py
```

List possible glove serial ports:

```bash
python test_glove_teleop.py --scan
```

Read RealForce hardware for five seconds, without sending commands to the hand:

```bash
python test_glove_teleop.py --serial --left-port /dev/ttyUSB0 --right-port /dev/ttyUSB1
```

`--baudrate 0` is the default and auto-scans the RealForce baudrate. If you know
the baudrate, pass it explicitly, for example `--baudrate 2000000`.

Read RealMCG UDP data for five seconds.  The MCG manual describes this as a
UDP broadcast from the upper-computer program, with default port `9011` and a
client handshake of `{"action":"CONNECT"}`:

```bash
python test_glove_teleop.py --mcg --mcg-host 127.0.0.1 --mcg-port 9011
```

Send live mapped commands to G20 hardware through the official SDK. Use G20 for
retargeting, while the SDK transport still uses `realhand.L20`:

```bash
python test_glove_teleop.py --serial --left-model g20 --right-model g20 \
  --left-port /dev/ttyUSB0 --right-port /dev/ttyUSB1 --sdk-send
```

The script now waits for a valid RealForce version and nonzero position frame
before it sends any SDK/CAN command.

For MCG to G20 SDK control:

```bash
python test_glove_teleop.py --mcg --left-model g20 --right-model g20 \
  --mcg-host 127.0.0.1 --mcg-port 9011 --sdk-send
```

If the MCG receiver or software pushes packets to a local UDP port:

```bash
python test_glove_teleop.py --mcg --mcg-host 127.0.0.1 --mcg-port 9011 --mcg-bind-port 9011
```

The USB-A receiver itself may enumerate as `/dev/ttyUSB*`, but the manual does
not document a raw serial frame protocol.  Use `--mcg-serial-port` only if the
receiver is confirmed to emit the same JSON frames directly.

```python
from realhand_pure_python import HandSide, RealForceSerialReader

reader = RealForceSerialReader(HandSide.RIGHT)
reader.open("/dev/ttyUSB0", 460800)
reader.start()

try:
    while True:
        snap = reader.snapshot()
        print(snap.side, snap.version, snap.positions)
finally:
    reader.stop()
```

Install dependency:

```bash
pip install pyserial pyyaml numpy
```

To read serial and retarget in one object:

```python
from realhand_pure_python import RealForceRetarget

retarget = RealForceRetarget(
    left_port="/dev/ttyUSB0",
    right_port="/dev/ttyUSB1",
    left_model="l25",
    right_model="l25",
)

retarget.start()
try:
    while True:
        command = retarget.poll()
        print("left", command.left_positions)
        print("right", command.right_positions)
finally:
    retarget.stop()
```

If another program already reads the glove, pass the raw 21-value arrays
directly:

```python
from realhand_pure_python import RealForceRetarget

retarget = RealForceRetarget(left_model="g20", right_model="g20")
command = retarget.map_glove_positions(left=left_glove, right=right_glove)
```

Use `calibration_path="path/to/calibration.yml"` only when the calibration was
recorded for the same glove/hand setup. The included sample is kept as a
reference file, not loaded automatically.

## RealMCG

```python
from realhand_pure_python import RealMCGRetarget

def send_to_hand(command):
    print("right", command.right_positions)
    print("left", command.left_positions)

retarget = RealMCGRetarget(
    host="192.168.11.88",
    port=8888,
    left_model="l25",
    right_model="l25",
    on_command=send_to_hand,
)

try:
    retarget.start()
    while True:
        pass
finally:
    retarget.stop()
```

If your RealMCG sender pushes UDP data to a fixed local port, pass
`bind_address=("0.0.0.0", YOUR_LOCAL_PORT)`.

## L20/G20 SDK Control

The GitHub RealHand SDK is used for the real L20/G20-compatible hardware:

```bash
pip install git+https://github.com/RealHand-Robotics/realbot-python-sdk.git
```

In this setup the left hand is `can0` and the right hand is `can1`. The current
GitHub SDK exposes G20-compatible hardware through `realhand.L20(side="left",
interface_name="can0")` and accepts 16 angles in the range `0-100`; the wrapper
converts the teleop 20-value `0-255` L20/G20 command list to that newer SDK
format. For G20 hardware, use `--left-model g20 --right-model g20` so retargeting
uses the G20 mapping.

Dry-run without sending CAN commands:

```bash
python control_l20_sdk.py --action open
```

Check/connect through the SDK:

```bash
python control_l20_sdk.py --action check --send
```

Send a gentle open command:

```bash
python control_l20_sdk.py --action open --send
```

Send a custom L20 pose:

```bash
python control_l20_sdk.py --action pose --send \
  --left-pose "250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250" \
  --right-pose "250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250"
```

The SDK opens SocketCAN at 1 Mbps. Bring `can0`/`can1` up before running the
script if they are not already up.
