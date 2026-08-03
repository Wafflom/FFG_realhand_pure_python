# Project Handoff

Context for whoever (person or AI assistant) picks this project up next. The
[README](README.md) covers *what the project is and how to run it*; this file
covers *where things stand, what was done, and what to do next*.

Last updated at commit `6e1a5d1` (v0.2.0).

---

## TL;DR

Glove-to-robot-hand teleoperation for RealHand **L6** and **L20** hands. Both are
verified working end to end over CAN. The current front is improving **L20 finger
tracking quality**, which is limited by glove sensor hardware, not by the control
code. The next concrete task is a **per-finger sensor-unmixing layer** (details
below).

## Verified working

- **L6** and **L20** hands driven live over CAN.
- Windows via PEAK PCAN-USB (`pcan` / `PCAN_USBBUS1`); Linux via SocketCAN
  (`can0`). Backend is auto-defaulted per OS.
- RealForce glove over USB serial; RealMCG glove over UDP (from the vendor app).
- Desktop GUI: device auto-detect, calibration wizard, live tuning, per-run log.

## How to get oriented fast

1. Read [README.md](README.md) — pipeline, install, project layout.
2. `python glove_teleop_gui.py` — the GUI is the fastest way to see the whole
   flow; its "Command that will run" box shows the exact CLI equivalent.
3. `test_glove_teleop.py --help` — every capability is a CLI flag; the GUI just
   composes these.
4. `git log --oneline` — each commit message explains a chunk of the history in
   detail.

## What has been done (high level)

The repo started as a coworker's pure-Python retarget library. Since then:

- **Windows support**: python-can backend passthrough (`--left/right-can-type`),
  a Windows COM-port fix, and a Windows-only workaround for the SDK's
  GIL-hogging CAN send-pacing busy-wait (was capping throughput at ~10–64
  frames/s and growing the send queue unboundedly).
- **Two platform-independent upstream bug fixes**: L20 crashed on the first
  frame with a calibration file (missing `_resolve_version_config`), and MCG
  mode failed with a single hand connected (`run_mcg` sent both).
- **Desktop GUI** (`glove_teleop_gui.py`) — front end over the CLI.
- **Latency/filter work**: input Kalman filter was over-damping rapid motion
  (~170 ms step response, erased fast shakes); softened to ~20 ms and made
  tunable. Fixed a Kalman memory leak. Output smoothing off by default.
- **Calibration**: illustrated pose wizard (canvas vector art). L20 captures 11
  poses (spread, separated root/tip flexion, extra thumb DOFs) spliced into the
  mapping range.
- **Tuning UI**: live-adjustable filters/rates + per-group motion exaggeration
  (thumb spread / thumb squeeze / fingers), each independent, `<1` damps `>1`
  exaggerates.

See git history for the specifics of any of these.

## OPEN WORK ITEM — L20 finger tracking

**Symptom (user's words):** the L20 "often displays curled finger tips when
they're straight, can't handle finger spread well, and finger flexion is poor."

**Root cause (measured from a real 11-pose calibration):** the RealForce glove's
per-segment finger sensors are physically **cross-coupled**.
- In a knuckle-bend (~90°) pose the "root" sensors registered only 13–30% of
  their range.
- In a fingertip-curl pose, tip motion **leaked 0.25–0.61 rad into the root
  sensors** — more than actual root bending produces.
- Finger spread is weakly sensed (ring spread ~0.055 rad ≈ 3°, near noise).

So the mapper's assumption that each sensor measures one joint is false on this
glove. Calibration mitigates but cannot separate signals the hardware doesn't
cleanly produce. (Note: the 11-pose L20 calibration was an attempt at this and
partially helped, but the coupling remains.)

**Planned fix, not yet started:**
1. **Sensor unmixing** — per finger, treat (root sensor, tip sensor) as a linear
   mix of (root angle, tip angle) and solve the 2×2 inverse. The mixing
   coefficients come from the calibration poses already captured (rest, fist,
   knuckle-bend "table", tip-curl "claw"). This directly attacks the phantom
   fingertip curl. Contained change, most likely in `joint_update` /
   `realhand_core_ex.map_glove_to_robot`.
2. **Fingertip IK (optional, stage 2)** — human-hand FK from unmixed angles →
   fingertip targets → analytic 2-link IK onto the L20 finger chains (URDFs are
   in `assets/`). Gives natural curl distribution. Pure numpy is fast enough at
   100 Hz.
3. **Spread** — given the weak sensor signal, drive spread mainly from the index
   sensor with the exaggeration knob and accept ring spread as approximate.

Start with step 1; it's the highest-value, lowest-risk change.

## Key operational facts (some not obvious from code)

- **Power: the hands need 24 V.** A 12 V supply runs the control board (CAN
  connects, light blinks) but **no motor moves** and every joint reports
  `MOTOR_COMM_ABNORMAL`. This wasted real debugging time — check it first.
- **CAN backend differs by OS**: `pcan`/`PCAN_USBBUS1` (Windows, needs the PEAK
  driver installed), `socketcan`/`can0` (Linux, no vendor driver needed). Two
  hands need two separate CAN channels.
- **Serial port numbers change on replug** — always Scan/Auto-detect.
- **Calibration is per person + per glove.** L6 = 5 poses, L20 = 11.
- **RealMCG has no calibration** — the vendor app pre-normalizes the data.
- **SDK scope**: realhand SDK 0.5.x supports O6/L6/L20 (+L25). **No L30/O30**
  yet — an L30 on site cannot be driven until the vendor ships SDK support.
- **Exaggeration defaults are gentle** (thumb 1.2, fingers 0.9) for healthy
  hardware; the old 2.0 thumb value was tuned against a near-dead thumb.

## How to verify changes

```bash
python -m pyflakes glove_teleop_gui.py test_glove_teleop.py realhand_pure_python
python test_glove_teleop.py --left-model l20 --right-model l20   # fake-data smoke
```

The mapping layer has been kept behavior-stable by capturing "golden" outputs
(mapper results for L6/L20 across calibrated and uncalibrated sweeps) before a
refactor and re-checking them byte-for-byte after. Recommend doing the same
before touching the mapping math — a small script that runs
`RealForceRetarget.map_glove_positions` over a fixed input sweep and hashes the
result is enough.

## Conventions

- Commit directly to `main` (this repo is already the user's own branch/fork).
- Keep changes behavior-preserving where possible; overrides (filters,
  exaggeration) are applied per-run so the mapper source stays untouched.
- The user is a self-described beginner programmer doing an internship — explain
  reasoning, not just conclusions.

## Environment

- Conda env `gloveTeleop` (Python 3.11). `conda env create -f
  environment-gloveTeleop.yml`, then `pip install -e .` and `pip install
  python-can` + the RealHand SDK. See [WINDOWS_SETUP.md](WINDOWS_SETUP.md).
