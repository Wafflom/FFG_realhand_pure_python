# RealHand Glove Teleop — Windows Setup

Quick setup for running the demo on a **Windows** desktop. Copy this whole
folder to the new machine first.

---

## 1. Install the prerequisites (once per machine)

### Miniconda (Python environment manager)

```powershell
winget install Anaconda.Miniconda3
```

Then **close and reopen the terminal** so `conda` is on the PATH. If it still
isn't recognized, run this once, reopen the terminal, and you should see
`(base)` in the prompt:

```powershell
& "$env:USERPROFILE\miniconda3\shell\condabin\conda-hook.ps1" ; conda init powershell
```

### Git (needed to fetch the RealHand SDK during install)

```powershell
winget install Git.Git
```

### Hardware drivers

- **Glove (CH340 USB-serial):** usually auto-installs when plugged in. If the
  glove never shows up as a `COM` port, install `CH341SER.EXE` (from the
  `mocap_driver` folder of the controller zip, or wch.cn).
- **CAN adapter (PEAK PCAN-USB):** install the PEAK driver from
  peak-system.com → *PCAN-USB* → Downloads. Without it the CAN adapter has no
  `PCAN_USBBUS1` channel.

---

## 2. Build the environment (once per machine)

Open a terminal **in this folder** and run:

```powershell
conda env create -f environment-gloveTeleop.yml
conda activate gloveTeleop
python -m pip install -e .
python -m pip install python-can
```

- `conda env create` makes the `gloveTeleop` environment (Python 3.11 + numpy,
  pyserial, pyyaml, and the RealHand SDK).
- `pip install -e .` installs this folder **editable**, so later code edits take
  effect with no reinstall.
- `python-can` is the CAN transport used by the SDK.

Verify:

```powershell
python glove_teleop_gui.py --help  ; python -c "import realhand, can; print('ok')"
```

---

## 3. Launch the GUI

From this folder, with the environment active:

```powershell
conda activate gloveTeleop
python glove_teleop_gui.py
```

Or in one line without activating first:

```powershell
conda run --no-capture-output -n gloveTeleop python glove_teleop_gui.py
```

---

## 4. Run a demo

In the window, work top to bottom:

| Step | Control |
|---|---|
| 1 | **Scan ports** → pick the glove (a CH340 device, e.g. `COM6`) |
| 2 | **Hands** → `Right only` unless two hands *and* two adapters |
| 3 | **Model** → `L6` or `L20`, matching the hardware |
| 4 | **Detect adapters** → confirms `pcan` / `PCAN_USBBUS1` |
| 5 | **Detect hand** → confirms the hand answers the selected model |
| 6 | **Start tracking** |

The **Command that will run** box shows the exact CLI equivalent.

### Calibration

`Run calibration wizard` captures the poses in plain English. It is per person,
per glove — run it once for whoever is demonstrating. Files are written to
`realhand_pure_python\config\`.

---

## Windows notes

- **CAN backend is `pcan`, channel `PCAN_USBBUS1`** — this is set by default on
  Windows. (Linux uses `socketcan` / `can0` instead.)
- **Power: the hand needs 24 V.** A 12 V supply runs the control board — CAN
  connects and the light blinks — but no joint moves and every joint reports
  `MOTOR_COMM_ABNORMAL`. Check this first if nothing responds.
- **COM port numbers change when you replug.** Always use **Scan ports** rather
  than assuming a previous name.
- Two hands need **two** PCAN channels; one PCAN-USB provides only
  `PCAN_USBBUS1`, so use `Right only` or `Left only` with a single adapter.

## Troubleshooting

Every run is logged to `glove_teleop_last_run.log` in this folder, and the GUI's
Output pane explains common failures in plain English.

| Symptom | Fix |
|---|---|
| `conda` not recognized | reopen the terminal after installing Miniconda |
| Glove not in Scan ports | install the CH340 driver; replug; Scan again |
| No `PCAN_USBBUS1` in Detect adapters | install the PEAK PCAN driver |
| Connects but nothing moves | 24 V supply; and press **Detect hand** for model match |
| Runs then drops to Idle | tick **Run continuously**, or set a positive seconds value |
