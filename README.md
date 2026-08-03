# RealHand Glove Teleoperation

Control [RealHand](https://github.com/RealHand-Robotics) dexterous robot hands in
real time from a data glove. Pure Python, no ROS. A data glove streams finger
poses, the software retargets them onto a robot hand's joints, and drives the
hand over CAN through the official RealHand SDK.

Two glove systems and several hand models are supported, with a desktop GUI that
handles device discovery, a guided calibration wizard, live response tuning, and
per-run diagnostics.

> **Picking this project up?** See **[HANDOFF.md](HANDOFF.md)** for current
> status, what's been done, the open work item, and gotchas.

---

## What it does

```
 GLOVE                    RETARGET                        ROBOT HAND
 ┌───────────┐   angles   ┌────────────────────────┐  cmd  ┌──────────┐
 │ RealForce │ ─────────▶ │ calibration + mapping  │ ────▶ │ L6 / L20 │
 │  (USB)    │            │ per-hand-model config  │  CAN  │  hand    │
 │ RealMCG   │ ─JSON/UDP▶ │ + Kalman/EMA filtering │       │          │
 └───────────┘            └────────────────────────┘       └──────────┘
```

- **RealForce glove** — read directly over USB serial (a binary framed protocol).
- **RealMCG glove** (MOTCAP G7s) — received as JSON over UDP from the vendor's
  upper-computer application.
- **Robot hands** — RealHand **L6** (6 DOF) and **L20** (20 DOF) over CAN.

## Features

- **Desktop GUI** (`glove_teleop_gui.py`) — a front end that builds and runs the
  teleop command, with auto-detection of gloves and CAN interfaces, live command
  preview, an output log with plain-English error explanations, and a per-run
  log file.
- **Guided calibration wizard** — illustrated, plain-English poses drawn on a
  canvas. L6 captures 5 poses; L20 captures 11 (adding finger spread, separated
  root/tip flexion, and extra thumb DOFs).
- **Live response tuning** — input Kalman filter, output smoothing, glove poll
  rate and CAN send rate are all adjustable from the GUI with hover tooltips.
- **Per-group motion exaggeration** — independently scale thumb-spread,
  thumb-squeeze and finger response to fit different hands and different people.
- **Cross-platform** — Windows (PEAK PCAN-USB via the `pcan` backend) and Linux
  (SocketCAN). A ready-to-run Linux demo bundle lives in a separate export.

---

## Requirements

- Python 3.10+ (3.11 recommended; the pinned conda env uses 3.11)
- `numpy`, `pyserial`, `PyYAML`, `python-can`, and `tkinter` (ships with most
  Python installs; on Debian/Ubuntu: `sudo apt install python3-tk`)
- The official RealHand SDK:
  `pip install git+https://github.com/RealHand-Robotics/realbot-python-sdk.git`
- Hardware drivers: CH340 USB-serial (for the glove) and, on Windows, the PEAK
  PCAN-USB driver (for CAN).

The SDK and CAN hardware are only needed to actually drive a hand. Reading a
glove, calibrating, and inspecting mapped commands work without them.

## Install

### Windows

See **[WINDOWS_SETUP.md](WINDOWS_SETUP.md)** for step-by-step setup on a fresh
machine. In short, from the repo root:

```powershell
conda env create -f environment-gloveTeleop.yml
conda activate gloveTeleop
python -m pip install -e .
python -m pip install python-can
```

### Linux

Use the conda environment as above, or a virtualenv:

```bash
python -m venv .venv && source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install -e .
```

Bring up CAN (the hands use 1 Mbps SocketCAN):

```bash
sudo ip link set can0 up type can bitrate 1000000
```

A PEAK PCAN-USB adapter needs **no vendor driver on Linux** — the kernel's
`peak_usb` module presents it as a normal SocketCAN interface, so use the
`socketcan` backend with channel `can0`.

## Run

```bash
python glove_teleop_gui.py
```

Then, working down the window:

1. **Scan ports / Auto-detect all** — finds the glove and CAN interface.
2. **Hands** — `Right only` unless you have two hands *and* two CAN channels.
3. **Model** — `L6` or `L20`, matching the connected hardware.
4. **Detect hand** — confirms the hand answers the selected model's protocol.
5. **Start tracking**.

The GUI shows the exact CLI it runs; everything is also available directly
through `test_glove_teleop.py` (`--help` lists all flags).

## Calibration

Click **Run calibration wizard** and follow the illustrated poses. Calibration
is per person and per glove — recapture when someone else wears the glove or the
fit changes. Files are written to `realhand_pure_python/config/`.

L20 records extra poses because it has more independent DOFs: finger spread,
knuckle-bend vs. tip-curl (separated), and additional thumb motions. These are
spliced into the mapping range so those channels track a real range of motion
rather than a guess.

## Tuning

All adjustable live from the GUI (with tooltips explaining each):

| Control | Effect |
|---|---|
| Input filter Q | Kalman responsiveness on raw glove values. Higher = snappier and keeps more rapid motion; lower = smoother but laggier; blank = bypass. |
| Output smoothing (alpha / max step) | Optional EMA + per-frame step cap on motor commands. Off by default. |
| Exaggeration (thumb spread / squeeze / fingers) | Per-group response scale. 1.0 = as calibrated, >1 exaggerates, <1 damps. |
| Glove poll / Send rate | Sampling and CAN command rates. |

---

## Hardware notes

- **Power: the hands need a 24 V supply.** A 12 V supply powers the control
  board — CAN connects and the status light blinks — but the motors do not move
  and every joint reports `MOTOR_COMM_ABNORMAL`. Check this first if nothing
  moves.
- **CAN backend:** `pcan` / `PCAN_USBBUS1` on Windows, `socketcan` / `can0` on
  Linux. Each hand opens its own bus, so driving two hands needs two channels.
- **Serial ports move when you replug.** Use Scan / Auto-detect rather than
  assuming a previous device name.

## Supported hands

Retargeting mappers exist for L6, L20, and several others (o6/o7/l7/l10/l25/g20).
The GUI intentionally offers only **L6** and **L20**, the models commissioned and
verified end to end. The SDK (0.5.x) drives O6/L6/L20 (+L25); there is no L30/O30
support in the SDK yet.

## Known limitations

- **RealMCG has no calibration stage.** The vendor application normalizes the
  glove data to 0–255 before it reaches this code, so any per-user fitting lives
  in that app, not here.
- **Glove sensor cross-coupling (L20).** On the RealForce glove the per-segment
  finger sensors are physically coupled — tip curl leaks into the "root" channel
  and spread is weakly sensed. Calibration mitigates this but cannot fully
  separate signals the hardware doesn't cleanly produce. A sensor-unmixing /
  IK layer is the planned next step.

## Project layout

```
glove_teleop_gui.py          desktop GUI (front end over the CLI)
test_glove_teleop.py         CLI entry point: scan, calibrate, serial/MCG teleop
realhand_pure_python/
  realforce.py               RealForce USB serial reader (binary protocol)
  realmcg.py                 RealMCG UDP/serial JSON reader and mapper
  realforce_retarget.py      orchestration + calibration loading
  realforce_hands/           per-hand-model mappers (l6, l20, ...)
  realforce_config/          per-model channel wiring and robot poses
  realhand_core.py           URDF joint limits, angle -> motor scaling
  realhand_core_ex.py        multi-state interpolation + input filtering
  realhand_filters.py        Kalman / EMA / Savitzky-Golay filters
  l20_sdk_controller.py      adapter to the official RealHand SDK (CAN)
  assets/                    robot hand URDFs
  config/                    hand_config.yml + bundled/sample calibration
control_l20_sdk.py           low-level SDK check/send helper
dump_serial.py               raw serial debug helper
```

## Development

```bash
python -m pyflakes glove_teleop_gui.py test_glove_teleop.py realhand_pure_python
python test_glove_teleop.py --left-model l6 --right-model l6   # fake-data smoke test
```

The mapping layer is behavior-checked against recorded golden outputs, so
refactors can be verified byte-for-byte identical.

## License

See [LICENSE](LICENSE).
