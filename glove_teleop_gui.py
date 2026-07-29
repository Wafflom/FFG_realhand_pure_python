#!/usr/bin/env python3
"""Desktop launcher for the RealHand pure Python glove teleop.

This is a front end only.  It does not import or modify any teleop logic; it
builds a `test_glove_teleop.py` command line from the form below and runs it as
a subprocess, streaming the output into the log pane.

Run it from the repo root inside the gloveTeleop environment:

    python glove_teleop_gui.py
"""

from __future__ import annotations

import queue
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import tkinter as tk
from tkinter import filedialog, ttk


REPO_ROOT = Path(__file__).resolve().parent
ENTRY_SCRIPT = REPO_ROOT / "test_glove_teleop.py"
# Everything shown in the Output pane is also appended here, so a run that ends
# too fast to read can still be inspected afterwards.
RUN_LOG = REPO_ROOT / "glove_teleop_last_run.log"

# Only the hands that are actually commissioned and verified end to end.
# The retarget layer has mappers for more models (o6/o7/l7/l10/l25/g20), but
# they are not set up to run, so they are deliberately not offered here.
HAND_MODELS = [
    ("l6", "L6 - 6 DOF"),
    ("l20", "L20 - 20 DOF"),
]

MODEL_LABELS = [label for _value, label in HAND_MODELS]
LABEL_TO_MODEL = {label: value for value, label in HAND_MODELS}

# python-can backend names the RealHand SDK accepts as interface_type.
# socketcan is Linux-only; pcan is the verified PEAK PCAN-USB path on Windows.
CAN_BACKENDS = [
    "socketcan",
    "pcan",
    "slcan",
    "kvaser",
    "ixxat",
    "vector",
    "canalystii",
    "seeedstudio",
    "virtual",
]

PAD = {"padx": 8, "pady": 4}

# Output-filter tuning passed to the CLI. The upstream values (0.5 / 20) are not
# edited in place - they are overridden per run, so the mapper source stays
# untouched and the Linux path is unaffected.
#
# max_step caps how many motor units the command may move per frame, so it sets
# hand speed. Measured for 90% of a full 0-255 travel at 100 Hz, and how much of
# a 6-unit sensor spike reaches the motor in one frame:
#
#   alpha 0.5 / step  20  ->  120 ms,  3 of 6 units   (upstream default)
#   alpha 0.7 / step  80  ->   40 ms,  4 of 6 units   <- used here
#   alpha 1.0 / step 255  ->   10 ms,  6 of 6 units   (no filtering at all)
#
# Lower max_step first if any channel becomes unstable: thumb abduction has
# extended_exp_factor 10 in l6_config.py, which amplifies extrapolation past the
# fist pose, so it is the most sensitive to a loose step limit.
SMOOTH_ALPHA = 0.7
SMOOTH_MAX_STEP = 80

# Thumb abduction range now comes from the calibration itself: the wizard
# captures dedicated Thumb Out (abducted) and Thumb In (adducted) poses, and
# those become the channel's endpoints. That replaces the old scale_factor
# fudge, which only stretched a range derived from rest-to-fist - a movement
# that barely touches the thumb side-swing sensor.

# How often the glove is asked for a position frame. The reader defaults to
# 60 Hz but the glove only delivered 32 frames/s at that rate; measured on this
# hardware, 120 Hz yields ~49 frames/s and anything above that gains nothing:
#   query 60 Hz -> 32.0 frames/s      query 200 Hz -> 48.7 frames/s
#   query 120 Hz -> 48.7 frames/s     query 400 Hz -> 49.0 frames/s
# That cuts the sampling contribution to latency from ~31 ms to ~20 ms.
SERIAL_QUERY_HZ = 120

# Latency is dominated by hardware, not the filter:
#   glove sample rate  ~31 ms  (measured 32 Hz)
#   send period        ~10 ms  (at 100 Hz)
#   CAN round trip     ~11 ms  (measured median)
# so roughly 40-50 ms end to end. Raising the send rate further does not help
# because no new glove data exists to send.

# (substring to match in output, plain-English explanation). Checked in order;
# the first match wins so more specific patterns come first.
ERROR_HINTS = [
    (
        "PcanCanInitializationError",
        "The PCAN adapter could not be opened.\n"
        "  - Another program or a leftover run may still hold it. Close other\n"
        "    teleop windows, or unplug and replug the adapter.\n"
        "  - Driving two hands needs two channels; one PCAN-USB only provides\n"
        "    PCAN_USBBUS1. Pick 'Left only' or 'Right only'.\n"
        "  - Press 'Detect adapters' to see what is actually present.",
    ),
    (
        "No module named 'realhand'",
        "The RealHand SDK is not installed in this environment.\n"
        "  Run: python -m pip install "
        "git+https://github.com/RealHand-Robotics/realbot-python-sdk.git",
    ),
    (
        "Access is denied",
        "The serial port is already in use.\n"
        "  Another teleop run, a terminal, or the vendor app is holding it.\n"
        "  Close the other program, then press 'Scan ports' again.",
    ),
    (
        "no RealForce glove opened",
        "The glove did not answer on the selected port.\n"
        "  - Check the port with 'Scan ports' (a CH340 device is the glove).\n"
        "  - Confirm the glove is powered and the USB cable is seated.\n"
        "  - Leave Baudrate at 0 so it auto-detects.",
    ),
    (
        "missing port(s)",
        "The configured serial port does not exist.\n"
        "  Press 'Scan ports' and pick one of the listed ports.",
    ),
    (
        "not sending CAN command",
        "Glove frames are not valid yet, so nothing is being sent to the hand.\n"
        "  This is normal for the first moment. If it persists, the glove is\n"
        "  connected but not streaming - check the port and side selection.",
    ),
    (
        "must contain",
        "The command length does not match the hand model.\n"
        "  The selected model does not match the connected hardware.\n"
        "  Press 'Detect hand' to see which protocol the hand answers.",
    ),
    (
        "Unsupported RealHand model",
        "That model is not supported by the retarget layer. Choose L6 or L20.",
    ),
    (
        "URDF path does not exist",
        "A robot description file is missing from the assets folder.\n"
        "  The repo checkout may be incomplete - try 'git status' in the repo.",
    ),
    (
        "Calibration file does not exist",
        "The calibration file path is wrong or the file was moved.\n"
        "  Use Browse to pick an existing file, or run the calibration wizard.",
    ),
    (
        "could not open the",
        "See the message above for which hand and channel failed.",
    ),
]

# The log pane is trimmed to this many lines. Without a cap the Text widget
# grows without bound and the whole UI becomes progressively sluggish.
MAX_LOG_LINES = 400

# A serial port as printed by --scan: a POSIX device node or a Windows COM name.
PORT_LINE_RE = re.compile(r"(?:/dev/[\w./-]+|COM\d+)")

# Linux reports a SocketCAN interface with this ARPHRD type in sysfs.
ARPHRD_CAN = "280"

# Calibration poses, in the order test_glove_teleop.py captures them. The pose
# keys ("original"/"opose"/"fist") are what the capture script prints.
CALIBRATION_POSES = [
    (
        "original",
        "1. Rest Pose",
        "Open your hand naturally.\n\n"
        "Fingers straight and relaxed, held flat.\n"
        "Do not stretch or tense them.",
    ),
    (
        "opose",
        "2. O-Pose (Pinch)",
        "Touch your thumb and index fingertips together\n"
        "to form a round 'O' shape.\n\n"
        "Let the other three fingers stay relaxed.",
    ),
    (
        "fist",
        "3. Fist",
        "Close your hand into a tight fist.\n\n"
        "Curl all fingers in and wrap your thumb\n"
        "across the front.",
    ),
    (
        "thumb_out",
        "4. Thumb Out",
        "Spread your thumb as far away from your\n"
        "palm as it will comfortably go.\n\n"
        "Keep the other fingers relaxed and still.",
    ),
    (
        "thumb_in",
        "5. Thumb In",
        "Bring your thumb in flat against the side\n"
        "of your palm.\n\n"
        "Keep the other fingers relaxed and still.",
    ),
]


class CalibrationWindow(tk.Toplevel):
    """Full-screen-ish guided calibration in English.

    Runs the same `--calibrate-realforce` capture as the CLI and translates its
    progress output into large, plain-English pose instructions. The capture
    script itself is untouched; this only reads its stdout.
    """

    def __init__(self, master: tk.Misc, args: list[str], output_path: str) -> None:
        super().__init__(master)
        self.title("Glove Calibration")
        self.geometry("640x620")
        self.transient(master)
        self.resizable(False, False)

        self.args = args
        self.output_path = output_path
        self.process: subprocess.Popen | None = None
        self.output_queue: queue.Queue[str | None] = queue.Queue()
        self.current_pose: str | None = None
        self.done = False
        self.failed = False

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._start()
        self.after(100, self._drain)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self, padding=16)
        outer.pack(fill="both", expand=True)

        ttk.Label(outer, text="Glove Calibration", font=("Segoe UI", 18, "bold")).pack(anchor="w")
        ttk.Label(
            outer,
            text="Three poses will be recorded. Hold each one still until it says Done.",
            foreground="#555555",
            wraplength=580,
        ).pack(anchor="w", pady=(2, 12))

        # Step indicators
        steps = ttk.Frame(outer)
        steps.pack(fill="x", pady=(0, 12))
        self.step_labels: dict[str, ttk.Label] = {}
        for index, (key, title, _desc) in enumerate(CALIBRATION_POSES):
            lbl = ttk.Label(
                steps,
                text=title,
                font=("Segoe UI", 10),
                foreground="#999999",
                padding=(8, 4),
            )
            lbl.grid(row=0, column=index, padx=(0, 12))
            self.step_labels[key] = lbl

        # Big instruction card
        card = tk.Frame(outer, bg="#f0f4f8", highlightbackground="#c8d4e0", highlightthickness=1)
        card.pack(fill="both", expand=True, pady=(0, 12))

        self.pose_title = tk.Label(
            card, text="Starting...", font=("Segoe UI", 24, "bold"), bg="#f0f4f8", fg="#1a3a5a"
        )
        self.pose_title.pack(pady=(28, 12))

        self.pose_desc = tk.Label(
            card,
            text="Connecting to the glove.",
            font=("Segoe UI", 13),
            bg="#f0f4f8",
            fg="#2a2a2a",
            justify="center",
        )
        self.pose_desc.pack(pady=(0, 20))

        self.status_big = tk.Label(
            card, text="", font=("Segoe UI", 17, "bold"), bg="#f0f4f8", fg="#a04000"
        )
        self.status_big.pack(pady=(0, 24))

        self.detail = ttk.Label(outer, text="", foreground="#555555", wraplength=580, justify="left")
        self.detail.pack(anchor="w", pady=(0, 8))

        bar = ttk.Frame(outer)
        bar.pack(fill="x")
        self.cancel_button = ttk.Button(bar, text="Cancel", command=self._on_close)
        self.cancel_button.pack(side="left")
        self.log_toggle = ttk.Button(bar, text="Show raw output", command=self._toggle_log)
        self.log_toggle.pack(side="left", padx=8)

        self.log_frame = ttk.Frame(outer)
        self.log = tk.Text(self.log_frame, height=7, wrap="word", font=("Consolas", 8))
        self.log.pack(fill="both", expand=True)
        self.log.configure(state="disabled")
        self._log_shown = False

    def _toggle_log(self) -> None:
        if self._log_shown:
            self.log_frame.pack_forget()
            self.log_toggle.configure(text="Show raw output")
        else:
            self.log_frame.pack(fill="both", expand=True, pady=(8, 0))
            self.log_toggle.configure(text="Hide raw output")
        self._log_shown = not self._log_shown

    def _start(self) -> None:
        try:
            self.process = subprocess.Popen(
                self.args,
                cwd=str(REPO_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as exc:
            self._set_pose_display("Failed to start", str(exc), "")
            return
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
        assert self.process is not None
        if self.process.stdout is not None:
            for line in self.process.stdout:
                self.output_queue.put(line)
        self.process.wait()
        self.output_queue.put(None)

    def _drain(self) -> None:
        try:
            while True:
                item = self.output_queue.get_nowait()
                if item is None:
                    self._finish()
                else:
                    self._handle_line(item)
        except queue.Empty:
            pass
        if not self.done:
            self.after(100, self._drain)

    def _handle_line(self, line: str) -> None:
        self._append_raw(line)
        text = line.strip()

        # Which pose is active? The capture script prints the pose key in English.
        for key, title, desc in CALIBRATION_POSES:
            if f"[calibration] {key}:" in text and self.current_pose != key:
                self.current_pose = key
                self._set_pose_display(title, desc, "Get ready...")
                self._mark_steps(key)
                break

        # Countdown, e.g. "... 7.0s ..." before capture starts.
        countdown = re.search(r"(\d+(?:\.\d+)?)s\s*(?:后开始采集|before)", text)
        if countdown and self.current_pose:
            self.status_big.configure(text=f"Get ready: {float(countdown.group(1)):.0f}s", fg="#a04000")
        elif "开始采集" in text or "sampling" in text.lower():
            self.status_big.configure(text="RECORDING - HOLD STILL", fg="#b00020")

        # Per-pose capture result, e.g. "[calibration] left original: {...}"
        result = re.search(r"\[calibration\]\s+(left|right)\s+(\w+):\s*(\{.*\})", text)
        if result:
            self.detail.configure(text=f"Captured {result.group(1)} {result.group(2)}: {result.group(3)}")
            self.status_big.configure(text="Captured", fg="#1a7f37")

        if "saved" in text and "[calibration]" in text:
            self._set_pose_display("Calibration complete", f"Saved to:\n{self.output_path}", "")
            self.status_big.configure(text="Done", fg="#1a7f37")
            for lbl in self.step_labels.values():
                lbl.configure(foreground="#1a7f37", font=("Segoe UI", 10, "bold"))

        if "no RealForce glove opened" in text or "missing port" in text:
            self._fail(
                "Glove not found",
                "The calibration could not open the glove serial port.\n"
                "Check the port selection and that nothing else is using it.",
            )
            return

        # Any Python error means the capture died; show it instead of leaving the
        # display frozen on its starting text.
        if "No serial ports were configured" in text:
            self._fail(
                "No glove port set",
                "The capture started with no serial port.\n\n"
                "Close this window, press 'Scan ports' on the main window,\n"
                "pick the glove port, then run the wizard again.",
            )
            return
        # The exception line arrives after the traceback header and carries the
        # useful text, so trigger on it rather than on "Traceback" itself. A
        # silent crash is still caught by the exit-code check in _finish().
        if re.match(r"^\s*\w*(Error|Exception):", text):
            self._fail("Calibration failed", text[:200])

    def _fail(self, title: str, detail: str) -> None:
        """Show a failure prominently and stop treating the run as in progress."""
        if self.failed:
            return
        self.failed = True
        self._set_pose_display(title, detail, "")
        self.status_big.configure(text="Failed", fg="#b00020")
        for lbl in self.step_labels.values():
            lbl.configure(foreground="#999999", font=("Segoe UI", 10))
        if not self._log_shown:
            self._toggle_log()

    def _mark_steps(self, active_key: str) -> None:
        reached = False
        for key, _title, _desc in CALIBRATION_POSES:
            lbl = self.step_labels[key]
            if key == active_key:
                lbl.configure(foreground="#0b5cad", font=("Segoe UI", 10, "bold"))
                reached = True
            elif not reached:
                lbl.configure(foreground="#1a7f37", font=("Segoe UI", 10))
            else:
                lbl.configure(foreground="#999999", font=("Segoe UI", 10))

    def _set_pose_display(self, title: str, desc: str, status: str) -> None:
        self.pose_title.configure(text=title)
        self.pose_desc.configure(text=desc)
        self.status_big.configure(text=status)

    def _append_raw(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text)
        excess = int(self.log.index("end-1c").split(".")[0]) - MAX_LOG_LINES
        if excess > 0:
            self.log.delete("1.0", f"{excess + 1}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _finish(self) -> None:
        self.done = True
        code = self.process.returncode if self.process else None
        self._append_raw(f"[gui] calibration finished (exit code {code})\n")
        self.cancel_button.configure(text="Close")
        if code == 0:
            if "complete" not in self.pose_title.cget("text").lower():
                self.status_big.configure(text="Finished", fg="#1a7f37")
        else:
            # Never leave the window sitting on its starting text after a failure.
            self._fail(
                "Calibration failed",
                f"The capture stopped early (exit code {code}).\n"
                "Press 'Show raw output' below for the details.",
            )

    def _on_close(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.done = True
        self.destroy()


class TeleopLauncher(ttk.Frame):
    def __init__(self, master: tk.Tk) -> None:
        super().__init__(master, padding=12)
        self.grid(row=0, column=0, sticky="nsew")
        master.columnconfigure(0, weight=1)
        master.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        self.process: subprocess.Popen | None = None
        self.output_queue: queue.Queue[str | None] = queue.Queue()
        # Set by start_calibration() before a capture run; never reuses the
        # calibration file chosen for playback.
        self._calibration_output = "realhand_pure_python/config/calibration_user.yml"
        # Error signatures already explained this run, so hints appear once.
        self._explained: set[str] = set()
        self._run_started = 0.0

        # --- form state -------------------------------------------------
        self.glove_system = tk.StringVar(value="realforce")
        self.hands = tk.StringVar(value="both")
        self.model_label = tk.StringVar(value=MODEL_LABELS[0])

        self.left_port = tk.StringVar()
        self.right_port = tk.StringVar()
        self.baudrate = tk.StringVar(value="0")

        self.mcg_host = tk.StringVar(value="127.0.0.1")
        self.mcg_port = tk.StringVar(value="9011")
        self.mcg_bind_port = tk.StringVar()

        self.use_calibration = tk.BooleanVar(value=False)
        self.calibration_path = tk.StringVar()

        # CAN is the only transport to the hands, so it is always enabled.
        # SocketCAN is Linux-only; on Windows the PEAK PCAN-USB adapter is the
        # verified path, so default the form to whatever this machine can use.
        if sys.platform == "win32":
            self.can_type = tk.StringVar(value="pcan")
            self.left_can = tk.StringVar(value="PCAN_USBBUS1")
            self.right_can = tk.StringVar(value="PCAN_USBBUS2")
        else:
            self.can_type = tk.StringVar(value="socketcan")
            self.left_can = tk.StringVar(value="can0")
            self.right_can = tk.StringVar(value="can1")

        self.reverse_thumb = tk.BooleanVar(value=False)
        self.no_smoothing = tk.BooleanVar(value=False)
        self.send_hz = tk.StringVar(value="100")
        self.continuous = tk.BooleanVar(value=True)
        self.seconds = tk.StringVar(value="30")
        self.verbose_log = tk.BooleanVar(value=False)

        self.status_text = tk.StringVar(value="Idle")

        self._build_ui()
        self._on_form_change()
        self.after(100, self._drain_output)
        # Auto-detect glove and CAN once on startup so the form comes up filled
        # in. Quiet so a headless machine does not spam the log.
        self.after(400, lambda: self.auto_detect(quiet=True))

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        row = 0

        header = ttk.Label(
            self,
            text="RealHand Glove Teleop",
            font=("Segoe UI", 16, "bold"),
        )
        header.grid(row=row, column=0, sticky="w")
        row += 1
        ttk.Label(
            self,
            text="Select your setup, then start tracking.",
            foreground="#555555",
        ).grid(row=row, column=0, sticky="w", pady=(0, 8))
        row += 1

        row = self._build_setup_box(row)
        row = self._build_connection_box(row)
        row = self._build_calibration_box(row)
        row = self._build_robot_box(row)
        row = self._build_command_box(row)
        row = self._build_actions(row)
        row = self._build_log(row)

    def _build_setup_box(self, row: int) -> int:
        box = ttk.LabelFrame(self, text="1. Setup", padding=8)
        box.grid(row=row, column=0, sticky="ew", pady=6)
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="Glove system").grid(row=0, column=0, sticky="w", **PAD)
        glove_frame = ttk.Frame(box)
        glove_frame.grid(row=0, column=1, sticky="w")
        ttk.Radiobutton(
            glove_frame,
            text="RealForce (USB serial)",
            value="realforce",
            variable=self.glove_system,
            command=self._on_form_change,
        ).grid(row=0, column=0, sticky="w", padx=(0, 16))
        ttk.Radiobutton(
            glove_frame,
            text="RealMCG (network)",
            value="mcg",
            variable=self.glove_system,
            command=self._on_form_change,
        ).grid(row=0, column=1, sticky="w")

        ttk.Label(box, text="Hands").grid(row=1, column=0, sticky="w", **PAD)
        hands_frame = ttk.Frame(box)
        hands_frame.grid(row=1, column=1, sticky="w")
        for index, (value, text) in enumerate(
            (("both", "Both"), ("left", "Left only"), ("right", "Right only"))
        ):
            ttk.Radiobutton(
                hands_frame,
                text=text,
                value=value,
                variable=self.hands,
                command=self._on_form_change,
            ).grid(row=0, column=index, sticky="w", padx=(0, 16))

        ttk.Label(box, text="Robot hand model").grid(row=2, column=0, sticky="w", **PAD)
        combo = ttk.Combobox(
            box,
            textvariable=self.model_label,
            values=MODEL_LABELS,
            state="readonly",
            width=24,
        )
        combo.grid(row=2, column=1, sticky="w", **PAD)
        combo.bind("<<ComboboxSelected>>", lambda _event: self._on_form_change())

        ttk.Button(box, text="Detect hand", command=self.detect_hand).grid(
            row=2, column=2, sticky="w", **PAD
        )

        ttk.Checkbutton(
            box,
            text="Reverse thumb abduction (tick if the thumb spreads the wrong way)",
            variable=self.reverse_thumb,
            command=self._on_form_change,
        ).grid(row=4, column=0, columnspan=3, sticky="w", **PAD)


        self.model_note = ttk.Label(box, text="", foreground="#a04000", wraplength=560, justify="left")
        self.model_note.grid(row=3, column=0, columnspan=3, sticky="w", **PAD)
        return row + 1

    def _build_connection_box(self, row: int) -> int:
        self.connection_box = ttk.LabelFrame(self, text="2. Connection", padding=8)
        self.connection_box.grid(row=row, column=0, sticky="ew", pady=6)
        self.connection_box.columnconfigure(1, weight=1)

        # RealForce serial widgets
        self.serial_frame = ttk.Frame(self.connection_box)
        self.serial_frame.columnconfigure(1, weight=1)

        ttk.Label(self.serial_frame, text="Left glove port").grid(row=0, column=0, sticky="w", **PAD)
        self.left_port_combo = ttk.Combobox(self.serial_frame, textvariable=self.left_port, width=28)
        self.left_port_combo.grid(row=0, column=1, sticky="ew", **PAD)

        ttk.Label(self.serial_frame, text="Right glove port").grid(row=1, column=0, sticky="w", **PAD)
        self.right_port_combo = ttk.Combobox(self.serial_frame, textvariable=self.right_port, width=28)
        self.right_port_combo.grid(row=1, column=1, sticky="ew", **PAD)

        ttk.Button(self.serial_frame, text="Scan ports", command=self.scan_ports).grid(
            row=0, column=2, sticky="ew", **PAD
        )
        ttk.Button(self.serial_frame, text="Auto-detect all", command=self.auto_detect).grid(
            row=1, column=2, sticky="ew", **PAD
        )

        ttk.Label(self.serial_frame, text="Baudrate").grid(row=2, column=0, sticky="w", **PAD)
        ttk.Entry(self.serial_frame, textvariable=self.baudrate, width=12).grid(
            row=2, column=1, sticky="w", **PAD
        )
        ttk.Label(
            self.serial_frame,
            text="0 = auto-detect",
            foreground="#555555",
        ).grid(row=2, column=2, sticky="w", **PAD)

        # RealMCG network widgets
        self.mcg_frame = ttk.Frame(self.connection_box)
        self.mcg_frame.columnconfigure(1, weight=1)

        ttk.Label(self.mcg_frame, text="MCG host").grid(row=0, column=0, sticky="w", **PAD)
        ttk.Entry(self.mcg_frame, textvariable=self.mcg_host, width=28).grid(
            row=0, column=1, sticky="w", **PAD
        )
        ttk.Label(self.mcg_frame, text="MCG port").grid(row=1, column=0, sticky="w", **PAD)
        ttk.Entry(self.mcg_frame, textvariable=self.mcg_port, width=12).grid(
            row=1, column=1, sticky="w", **PAD
        )
        ttk.Label(self.mcg_frame, text="Local bind port").grid(row=2, column=0, sticky="w", **PAD)
        ttk.Entry(self.mcg_frame, textvariable=self.mcg_bind_port, width=12).grid(
            row=2, column=1, sticky="w", **PAD
        )
        ttk.Label(
            self.mcg_frame,
            text="optional, only if the sender pushes to a fixed port",
            foreground="#555555",
        ).grid(row=2, column=2, sticky="w", **PAD)

        for var in (
            self.left_port,
            self.right_port,
            self.baudrate,
            self.mcg_host,
            self.mcg_port,
            self.mcg_bind_port,
        ):
            var.trace_add("write", lambda *_args: self._update_command_preview())
        return row + 1

    def _build_calibration_box(self, row: int) -> int:
        box = ttk.LabelFrame(self, text="3. Calibration", padding=8)
        box.grid(row=row, column=0, sticky="ew", pady=6)
        box.columnconfigure(1, weight=1)

        ttk.Checkbutton(
            box,
            text="Use a calibration file",
            variable=self.use_calibration,
            command=self._on_form_change,
        ).grid(row=0, column=0, sticky="w", **PAD)

        self.calibration_entry = ttk.Entry(box, textvariable=self.calibration_path)
        self.calibration_entry.grid(row=0, column=1, sticky="ew", **PAD)
        self.calibration_browse = ttk.Button(box, text="Browse", command=self.browse_calibration)
        self.calibration_browse.grid(row=0, column=2, **PAD)

        ttk.Button(box, text="Run calibration wizard", command=self.start_calibration).grid(
            row=1, column=0, sticky="w", **PAD
        )
        ttk.Label(
            box,
            text="Captures the open / O-pose / fist poses and writes a new calibration file.",
            foreground="#555555",
        ).grid(row=1, column=1, columnspan=2, sticky="w", **PAD)

        self.calibration_path.trace_add("write", lambda *_args: self._update_command_preview())
        return row + 1

    def _build_robot_box(self, row: int) -> int:
        box = ttk.LabelFrame(self, text="4. Robot hands (CAN)", padding=8)
        box.grid(row=row, column=0, sticky="ew", pady=6)
        box.columnconfigure(3, weight=1)

        ttk.Label(
            box,
            text="The hands are driven over CAN. Set the adapter and channel for each hand in use.",
            foreground="#555555",
            wraplength=620,
            justify="left",
        ).grid(row=0, column=0, columnspan=4, sticky="w", **PAD)

        ttk.Label(box, text="CAN adapter").grid(row=1, column=0, sticky="w", **PAD)
        self.can_type_combo = ttk.Combobox(
            box,
            textvariable=self.can_type,
            values=CAN_BACKENDS,
            state="readonly",
            width=14,
        )
        self.can_type_combo.grid(row=1, column=1, sticky="w", **PAD)
        self.can_type_combo.bind("<<ComboboxSelected>>", lambda _e: self._on_form_change())
        self.detect_can_button = ttk.Button(box, text="Detect adapters", command=self.detect_can)
        self.detect_can_button.grid(row=1, column=2, sticky="w", **PAD)

        ttk.Label(box, text="Left channel").grid(row=2, column=0, sticky="w", **PAD)
        self.left_can_entry = ttk.Entry(box, textvariable=self.left_can, width=18)
        self.left_can_entry.grid(row=2, column=1, sticky="w", **PAD)
        ttk.Label(box, text="Right channel").grid(row=2, column=2, sticky="w", **PAD)
        self.right_can_entry = ttk.Entry(box, textvariable=self.right_can, width=18)
        self.right_can_entry.grid(row=2, column=3, sticky="w", **PAD)

        self.can_note = ttk.Label(box, text="", foreground="#a04000", wraplength=620, justify="left")
        self.can_note.grid(row=3, column=0, columnspan=4, sticky="w", **PAD)

        ttk.Checkbutton(
            box,
            text="TEST: no smoothing at all (raw glove values straight to the motors)",
            variable=self.no_smoothing,
            command=self._on_form_change,
        ).grid(row=4, column=0, columnspan=4, sticky="w", **PAD)

        ttk.Label(box, text="Send rate (Hz)").grid(row=5, column=0, sticky="w", **PAD)
        ttk.Entry(box, textvariable=self.send_hz, width=12).grid(row=5, column=1, sticky="w", **PAD)

        ttk.Checkbutton(
            box,
            text="Run continuously",
            variable=self.continuous,
            command=self._on_form_change,
        ).grid(row=5, column=2, sticky="w", **PAD)
        self.seconds_entry = ttk.Entry(box, textvariable=self.seconds, width=12)
        self.seconds_entry.grid(row=5, column=3, sticky="w", **PAD)

        for var in (self.send_hz, self.seconds, self.left_can, self.right_can):
            var.trace_add("write", lambda *_args: self._update_command_preview())
        return row + 1

    def _build_command_box(self, row: int) -> int:
        box = ttk.LabelFrame(self, text="Command that will run", padding=8)
        box.grid(row=row, column=0, sticky="ew", pady=6)
        box.columnconfigure(0, weight=1)

        self.command_preview = tk.Text(box, height=4, wrap="word", font=("Consolas", 9))
        self.command_preview.grid(row=0, column=0, sticky="ew")
        self.command_preview.configure(state="disabled", background="#f4f4f4")

        ttk.Button(box, text="Copy", command=self.copy_command).grid(row=0, column=1, sticky="n", padx=(8, 0))
        return row + 1

    def _build_actions(self, row: int) -> int:
        bar = ttk.Frame(self)
        bar.grid(row=row, column=0, sticky="ew", pady=6)

        self.start_button = ttk.Button(bar, text="Start tracking", command=self.start_teleop)
        self.start_button.grid(row=0, column=0, padx=(0, 8))

        self.stop_button = ttk.Button(bar, text="Stop", command=self.stop_process, state="disabled")
        self.stop_button.grid(row=0, column=1, padx=(0, 8))

        ttk.Button(bar, text="Clear log", command=self.clear_log).grid(row=0, column=2, padx=(0, 8))
        ttk.Checkbutton(
            bar,
            text="Verbose log",
            variable=self.verbose_log,
            command=self._update_command_preview,
        ).grid(row=0, column=3, padx=(0, 16))

        ttk.Label(bar, textvariable=self.status_text, font=("Segoe UI", 10, "bold")).grid(
            row=0, column=4, sticky="w"
        )
        return row + 1

    def _build_log(self, row: int) -> int:
        box = ttk.LabelFrame(self, text="Output", padding=8)
        box.grid(row=row, column=0, sticky="nsew", pady=6)
        box.columnconfigure(0, weight=1)
        box.rowconfigure(0, weight=1)
        self.rowconfigure(row, weight=1)

        self.log = tk.Text(box, height=14, wrap="none", font=("Consolas", 9))
        self.log.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(box, orient="vertical", command=self.log.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scroll.set, state="disabled")
        return row + 1

    # ------------------------------------------------------------------
    # Form logic
    # ------------------------------------------------------------------
    def _on_form_change(self) -> None:
        is_realforce = self.glove_system.get() == "realforce"

        self.serial_frame.grid_forget()
        self.mcg_frame.grid_forget()
        if is_realforce:
            self.serial_frame.grid(row=0, column=0, sticky="ew")
        else:
            self.mcg_frame.grid(row=0, column=0, sticky="ew")

        hands = self.hands.get()
        self.left_port_combo.configure(state="normal" if hands in ("both", "left") else "disabled")
        self.right_port_combo.configure(state="normal" if hands in ("both", "right") else "disabled")

        cal_state = "normal" if self.use_calibration.get() else "disabled"
        self.calibration_entry.configure(state=cal_state)
        self.calibration_browse.configure(state=cal_state)

        # CAN is always active; only the unused side's channel is greyed out.
        self.left_can_entry.configure(state="normal" if hands in ("both", "left") else "disabled")
        self.right_can_entry.configure(state="normal" if hands in ("both", "right") else "disabled")
        self.can_type_combo.configure(state="readonly")
        self.detect_can_button.configure(state="normal")

        self.can_note.configure(text=self._can_warning())

        self.seconds_entry.configure(state="disabled" if self.continuous.get() else "normal")
        self._update_command_preview()

    def _can_warning(self) -> str:
        backend = self.can_type.get()
        if backend == "socketcan" and sys.platform == "win32":
            return (
                "SocketCAN only exists on Linux. On Windows choose 'pcan' (PEAK PCAN-USB) "
                "and use channel names like PCAN_USBBUS1."
            )
        if backend == "pcan" and self.hands.get() == "both":
            return (
                "Each hand opens its own CAN bus, so two hands need two PCAN channels. "
                "One PCAN-USB adapter exposes only PCAN_USBBUS1 - the second hand will "
                "fail unless a second adapter is plugged in. Use 'Detect adapters' to check."
            )
        return ""

    def _model(self) -> str:
        return LABEL_TO_MODEL[self.model_label.get()]

    def detect_hand(self) -> None:
        """Ask the hand on the CAN bus which protocol it answers.

        A hand only replies to its own model's messages, so a mismatched model
        silently does nothing - commands go out and are ignored. This probes
        read-only (no motion) and reports what actually responded.
        """
        channel = (self.left_can.get() if self.hands.get() == "left" else self.right_can.get()).strip()
        side = "left" if self.hands.get() == "left" else "right"
        if not channel:
            self.append_log("[gui] set a CAN channel first (use 'Detect adapters').\n")
            return

        self.append_log(f"[gui] probing {side} hand on {channel} ({self.can_type.get()})...\n")
        script = (
            "import time,sys\n"
            "from realhand import L6, L20\n"
            f"ch,ty,sd={channel!r},{self.can_type.get()!r},{side!r}\n"
            "for name,cls in (('l6',L6),('l20',L20)):\n"
            "    h=None\n"
            "    try:\n"
            "        h=cls(side=sd,interface_name=ch,interface_type=ty)\n"
            "        time.sleep(1.5)\n"
            "        print(f'{name}={h.get_snapshot().angle is not None}')\n"
            "    except Exception as e:\n"
            "        print(f'{name}=ERROR {type(e).__name__}')\n"
            "    finally:\n"
            "        if h is not None:\n"
            "            try: h.close()\n"
            "            except Exception: pass\n"
            "        time.sleep(0.4)\n"
        )
        try:
            result = subprocess.run(
                [sys.executable, "-u", "-c", script],
                cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=60,
            )
        except Exception as exc:
            self.append_log(f"[gui] probe failed: {exc}\n")
            return

        out = result.stdout or ""
        self.append_log(out)
        if result.stderr:
            self.append_log(result.stderr[-500:])

        responded = [line.split("=")[0] for line in out.splitlines() if line.strip().endswith("=True")]
        if not responded:
            self.model_note.configure(
                text="No hand responded on this CAN channel. Check power, wiring and the channel name."
            )
            self.status_text.set("No hand detected")
            return

        detected = responded[0]
        self.status_text.set(f"Hand speaks {detected.upper()}")
        current = self._model()
        expected_sdk = "l6" if current == "l6" else "l20"
        if expected_sdk == detected:
            self.model_note.configure(text=f"Hand confirmed: {detected.upper()} protocol matches '{current}'.")
        else:
            self.model_note.configure(
                text=f"MISMATCH: the hand answers {detected.upper()}, but the selected model "
                     f"'{current}' sends {expected_sdk.upper()} messages. The hand will ignore "
                     f"them and stay still. Select an {detected.upper()} model."
            )

    def auto_detect(self, *, quiet: bool = False) -> None:
        """One click: find the glove and the CAN interface and fill the form."""
        if not quiet:
            self.append_log("[gui] auto-detecting glove and CAN...\n")
        ports = self.scan_ports(quiet=quiet)
        self.detect_can(quiet=quiet)
        if quiet:
            bits = []
            if ports:
                bits.append(f"glove {ports[0]}")
            if self.left_can.get():
                bits.append(f"CAN {self.can_type.get()}:{self.left_can.get()}")
            if bits:
                self.status_text.set("Detected " + ", ".join(bits))

    @staticmethod
    def _list_socketcan_interfaces() -> list[dict]:
        """List Linux SocketCAN interfaces straight from sysfs.

        python-can's detection can miss an interface that exists but is still
        down (not yet `ip link up`), which is exactly the state before setup_can
        runs. Reading /sys/class/net catches those too. Non-Linux returns [].
        """
        found = []
        net = Path("/sys/class/net")
        if not net.is_dir():
            return found
        for iface in sorted(net.iterdir()):
            try:
                if (iface / "type").read_text().strip() != ARPHRD_CAN:
                    continue
            except OSError:
                continue
            state = ""
            try:
                state = (iface / "operstate").read_text().strip()
            except OSError:
                pass
            found.append({"interface": "socketcan", "channel": iface.name,
                          "device_name": f"SocketCAN ({state or 'unknown'})"})
        return found

    def detect_can(self, *, quiet: bool = False) -> None:
        """Find CAN interfaces. On Linux the hand uses SocketCAN (can0/can1)."""
        if not quiet:
            self.append_log("[gui] detecting CAN adapters...\n")
        can_configs: list[dict] = []

        # Linux: enumerate SocketCAN interfaces directly, including down ones.
        socketcan = self._list_socketcan_interfaces()
        can_configs.extend(socketcan)

        # Everything python-can can see (PCAN etc.), skipping duplicates.
        try:
            import can
            for cfg in can.detect_available_configs():
                if cfg.get("interface") == "serial":
                    continue
                key = (cfg.get("interface"), str(cfg.get("channel")))
                if key not in {(c["interface"], str(c["channel"])) for c in can_configs}:
                    can_configs.append(cfg)
        except ImportError as exc:
            if not socketcan:
                self.append_log(f"[gui] python-can not available: {exc}\n")
        except Exception as exc:
            self.append_log(f"[gui] python-can detection failed: {exc}\n")

        if not can_configs:
            if not quiet:
                self.append_log(
                    "[gui] no CAN interfaces found.\n"
                    "      On Linux, plug in the adapter and run SETUP.sh (or:\n"
                    "      sudo ip link set can0 up type can bitrate 1000000).\n"
                )
                self.status_text.set("No CAN adapters")
            return

        if not quiet:
            for config in can_configs:
                name = config.get("device_name", "")
                self.append_log(
                    f"  interface={config.get('interface')} channel={config.get('channel')}"
                    + (f" ({name})" if name else "")
                    + "\n"
                )

        real = [c for c in can_configs if c.get("interface") != "virtual"]
        if real:
            # Prefer SocketCAN when present: it is what the hand uses on Linux.
            real.sort(key=lambda c: 0 if c["interface"] == "socketcan" else 1)
            self.can_type.set(real[0]["interface"])
            self.left_can.set(str(real[0]["channel"]))
            # With a single adapter, point both sides at it so Left-only and
            # Right-only both work; running Both still needs a second adapter
            # and _can_warning() says so.
            self.right_can.set(str(real[1]["channel"] if len(real) > 1 else real[0]["channel"]))
            self.status_text.set(f"Found {len(real)} CAN interface(s)")
        else:
            self.status_text.set("Only virtual CAN found")
        self._on_form_change()

    def build_command(self, *, calibrate: bool = False) -> list[str]:
        """Assemble the test_glove_teleop.py argument list from the form."""
        model = self._model()
        hands = self.hands.get()
        args = [sys.executable, "-u", str(ENTRY_SCRIPT)]

        if calibrate:
            args.append("--calibrate-realforce")
        elif self.glove_system.get() == "realforce":
            args.append("--serial")
        else:
            args.append("--mcg")

        args += ["--left-model", model, "--right-model", model]

        if self.glove_system.get() == "realforce" or calibrate:
            if hands in ("both", "left") and self.left_port.get().strip():
                args += ["--left-port", self.left_port.get().strip()]
            if hands in ("both", "right") and self.right_port.get().strip():
                args += ["--right-port", self.right_port.get().strip()]
            baud = self.baudrate.get().strip() or "0"
            args += ["--baudrate", baud]
            args += ["--serial-query-hz", str(SERIAL_QUERY_HZ)]
        else:
            args += ["--mcg-host", self.mcg_host.get().strip() or "127.0.0.1"]
            args += ["--mcg-port", self.mcg_port.get().strip() or "9011"]
            if self.mcg_bind_port.get().strip():
                args += ["--mcg-bind-port", self.mcg_bind_port.get().strip()]

        if calibrate:
            # Written by start_calibration() from an explicit Save-as dialog so a
            # capture can never overwrite the calibration file selected above.
            args += ["--calibration-output", self._calibration_output]
            args += ["--calibration-sample-seconds", "3", "--calibration-ready-seconds", "8"]
            return args

        # The MCG path has no calibration stage, so these only apply to RealForce.
        if self.glove_system.get() == "realforce":
            if self.use_calibration.get() and self.calibration_path.get().strip():
                args += ["--calibration-path", self.calibration_path.get().strip()]
            args.append("--no-load-sample-calibration")

        # The hands only speak CAN, so every teleop run sends over CAN.
        # Flags mirror REALFORCE_L6_TELEOP_README.md, including the explicit
        # --left/right-sdk-model the README uses instead of relying on "auto".
        can_type = self.can_type.get()
        sdk_model = "l6" if model == "l6" else "l20"
        args.append("--sdk-send")
        args += ["--left-sdk-model", sdk_model, "--right-sdk-model", sdk_model]
        args += ["--left-can", self.left_can.get().strip() if hands in ("both", "left") else ""]
        args += ["--right-can", self.right_can.get().strip() if hands in ("both", "right") else ""]
        args += ["--left-can-type", can_type, "--right-can-type", can_type]

        if self.no_smoothing.get():
            # Full bypass: _apply_smooth returns the raw motor values untouched.
            args.append("--no-smoothing")
        else:
            args += ["--smooth-alpha", str(SMOOTH_ALPHA),
                     "--smooth-max-step", str(SMOOTH_MAX_STEP)]
        args.append("--reverse-thumb-abduction" if self.reverse_thumb.get()
                    else "--no-reverse-thumb-abduction")
        args += ["--send-hz", self.send_hz.get().strip() or "100"]
        # Status printing is purely diagnostic and every line costs GUI redraw
        # time, so keep it low unless the user asks for detail.
        args += ["--print-hz", "5" if self.verbose_log.get() else "1"]
        args += ["--seconds", "999999" if self.continuous.get() else (self.seconds.get().strip() or "30")]
        return args

    def _update_command_preview(self) -> None:
        try:
            args = self.build_command()
        except Exception as exc:  # pragma: no cover - defensive
            text = f"(cannot build command: {exc})"
        else:
            text = " ".join(_quote(part) for part in args)
        self.command_preview.configure(state="normal")
        self.command_preview.delete("1.0", "end")
        self.command_preview.insert("1.0", text)
        self.command_preview.configure(state="disabled")

    def copy_command(self) -> None:
        self.clipboard_clear()
        self.clipboard_append(self.command_preview.get("1.0", "end").strip())
        self.status_text.set("Command copied")

    def browse_calibration(self) -> None:
        path = filedialog.askopenfilename(
            title="Select calibration file",
            initialdir=str(REPO_ROOT / "realhand_pure_python" / "config"),
            filetypes=[("Calibration", "*.yml *.yaml *.json"), ("All files", "*.*")],
        )
        if path:
            self.calibration_path.set(path)

    # ------------------------------------------------------------------
    # Process control
    # ------------------------------------------------------------------
    def _find_serial_ports(self) -> list[str]:
        """List glove serial ports in-process.

        Calling the library directly avoids launching a subprocess and scraping
        its stdout, which was fragile: a stray warning line shifted the parse
        and dropped a real port. Falls back to the --scan subprocess only if the
        import is somehow unavailable.
        """
        try:
            from realhand_pure_python.realforce import scan_usb_serial_ports
            return scan_usb_serial_ports()
        except Exception as exc:
            self.append_log(f"[gui] direct scan unavailable ({exc}); using --scan\n")
        ports: list[str] = []
        try:
            result = subprocess.run(
                [sys.executable, "-u", str(ENTRY_SCRIPT), "--scan"],
                cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=30,
            )
            for line in (result.stdout or "").splitlines():
                candidate = line.strip()
                if PORT_LINE_RE.fullmatch(candidate) and candidate not in ports:
                    ports.append(candidate)
        except Exception as exc:
            self.append_log(f"[gui] scan failed: {exc}\n")
        return ports

    def scan_ports(self, *, quiet: bool = False) -> list[str]:
        if not quiet:
            self.append_log("[gui] scanning serial ports...\n")
        ports = self._find_serial_ports()
        self.left_port_combo.configure(values=ports)
        self.right_port_combo.configure(values=ports)
        if ports:
            self.append_log(f"[gui] serial ports: {', '.join(ports)}\n")
            if not self.left_port.get():
                self.left_port.set(ports[0])
            if not self.right_port.get() and len(ports) > 1:
                self.right_port.set(ports[1])
            self.status_text.set(f"Found {len(ports)} port(s)")
        elif not quiet:
            self.append_log(
                "[gui] no glove serial port found.\n"
                "      Plug the glove in. On Linux it is /dev/ttyUSB0; if it is\n"
                "      missing, check permissions (SETUP.sh adds a udev rule) or\n"
                "      run: ls -l /dev/ttyUSB*\n"
            )
            self.status_text.set("No ports found")
        return ports

    def start_teleop(self) -> None:
        problem = self._validate()
        if problem:
            self.append_log(f"\n[gui] cannot start: {problem}\n")
            self.status_text.set("Check settings")
            return
        self._launch(self.build_command(), "Tracking")

    def _validate_ports(self) -> str | None:
        """Glove serial ports needed for the selected hands."""
        if self.glove_system.get() != "realforce":
            return None
        hands = self.hands.get()
        missing = []
        if hands in ("both", "left") and not self.left_port.get().strip():
            missing.append("left")
        if hands in ("both", "right") and not self.right_port.get().strip():
            missing.append("right")
        if missing:
            return (
                f"No serial port set for the {' and '.join(missing)} glove. "
                f"Press 'Scan ports' and pick a port (the glove shows up as a CH340 device), "
                f"or change Hands to match the glove you have connected."
            )
        return None

    def _validate(self) -> str | None:
        """Catch setups that would fail at the driver level with a cryptic error."""
        port_problem = self._validate_ports()
        if port_problem:
            return port_problem

        if not self.continuous.get():
            raw = self.seconds.get().strip()
            try:
                seconds = float(raw)
            except ValueError:
                return (f"Run duration {raw!r} is not a number. Enter a positive value, "
                        f"or tick 'Run continuously'.")
            if seconds <= 0:
                return (f"Run duration is {raw}, so tracking would stop immediately. "
                        f"Enter a positive value, or tick 'Run continuously'.")

        try:
            if float(self.send_hz.get().strip() or "0") <= 0:
                return "Send rate must be greater than 0 Hz."
        except ValueError:
            return f"Send rate {self.send_hz.get()!r} is not a number."

        hands = self.hands.get()
        if hands in ("both", "left") and not self.left_can.get().strip():
            return "No CAN channel set for the left hand. Use 'Detect adapters'."
        if hands in ("both", "right") and not self.right_can.get().strip():
            return "No CAN channel set for the right hand. Use 'Detect adapters'."
        if self.hands.get() == "both":
            left, right = self.left_can.get().strip(), self.right_can.get().strip()
            if left and left == right:
                return (
                    f"Both hands are set to the same CAN channel ({left}). Each hand opens "
                    f"its own bus, so the second one will fail. Use two channels, or pick "
                    f"'Left only' / 'Right only'."
                )
        return None

    def start_calibration(self) -> None:
        if self.glove_system.get() != "realforce":
            self.append_log("[gui] calibration only applies to RealForce gloves.\n")
            return

        # Without a port the capture dies immediately inside RealForceRetarget,
        # so refuse here with something actionable instead.
        problem = self._validate_ports()
        if problem:
            self.append_log(f"\n[gui] cannot calibrate: {problem}\n")
            self.status_text.set("Check settings")
            return

        model = self._model()
        output = filedialog.asksaveasfilename(
            title="Save new calibration as",
            initialdir=str(REPO_ROOT / "realhand_pure_python" / "config"),
            initialfile=f"calibration_{model}_{self.hands.get()}.yml",
            defaultextension=".yml",
            filetypes=[("Calibration", "*.yml *.yaml *.json"), ("All files", "*.*")],
        )
        if not output:
            self.append_log("[gui] calibration cancelled.\n")
            return

        self._calibration_output = output
        CalibrationWindow(self.winfo_toplevel(), self.build_command(calibrate=True), output)
        # Offer the freshly captured file for the next teleop run.
        self.calibration_path.set(output)
        self.use_calibration.set(True)
        self._on_form_change()

    def _launch(self, args: list[str], status: str) -> None:
        if self.process is not None and self.process.poll() is None:
            self.append_log("[gui] a run is already active; stop it first.\n")
            return

        self._explained.clear()   # hints should reappear for each fresh run
        self._run_started = time.monotonic()
        # Drop anything left from a previous run. A stale end-of-stream sentinel
        # here would immediately mark this new run as finished.
        while True:
            try:
                self.output_queue.get_nowait()
            except queue.Empty:
                break
        try:
            RUN_LOG.write_text("", encoding="utf-8")     # one run per file
        except Exception:
            pass
        self.append_log(f"\n[gui] running: {' '.join(_quote(part) for part in args)}\n")
        try:
            self.process = subprocess.Popen(
                args,
                cwd=str(REPO_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as exc:
            self.append_log(f"[gui] failed to start: {exc}\n")
            return

        self.status_text.set(status)
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        threading.Thread(target=self._reader_thread, args=(self.process,), daemon=True).start()

    def _reader_thread(self, process: subprocess.Popen) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                self.output_queue.put(line)
        process.wait()
        self.output_queue.put(None)

    def _drain_output(self) -> None:
        # Any escaping exception would stop this from rescheduling itself and
        # the Output pane would silently freeze, so catch everything.
        try:
            while True:
                try:
                    item = self.output_queue.get_nowait()
                except queue.Empty:
                    break
                if item is None:
                    self._on_process_finished()
                else:
                    self.append_log(item)
                    self._explain(item)
        except Exception as exc:
            try:
                self.append_log(f"[gui] output pump error: {type(exc).__name__}: {exc}\n")
            except Exception:
                pass
        self.after(100, self._drain_output)

    def _explain(self, line: str) -> None:
        """Translate a known failure signature into plain English, once each."""
        for signature, explanation in ERROR_HINTS:
            if signature not in line:
                continue
            if signature in self._explained:
                return
            self._explained.add(signature)
            self.append_log(f"\n  >> WHAT THIS MEANS:\n  {explanation}\n\n".replace("\n  ", "\n     "))
            return

    def _on_process_finished(self) -> None:
        code = self.process.returncode if self.process else None
        if code == 0:
            ran = time.monotonic() - self._run_started
            self.append_log(f"[gui] finished normally after {ran:.1f}s (exit code 0)\n")
            if ran < 5.0:
                self.append_log(
                    "      That is much shorter than expected. Most likely the run\n"
                    "      duration is small - tick 'Run continuously', or set a larger\n"
                    "      value in the seconds box next to it.\n"
                )
        elif code is None:
            self.append_log("[gui] finished (exit code unknown)\n")
        else:
            self.append_log(
                f"[gui] STOPPED WITH AN ERROR (exit code {code}).\n"
                f"      Scroll up for the traceback; the last few lines name the cause.\n"
                f"      If no hint appeared above, turn on 'Verbose log' and run again.\n"
            )
        self.status_text.set("Idle")
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.process = None

    def stop_process(self) -> None:
        if self.process is None or self.process.poll() is not None:
            return
        self.append_log("[gui] stopping...\n")
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()

    # ------------------------------------------------------------------
    # Log helpers
    # ------------------------------------------------------------------
    def append_log(self, text: str) -> None:
        try:
            with RUN_LOG.open("a", encoding="utf-8", errors="replace") as handle:
                handle.write(text)
        except Exception:
            pass    # logging to disk must never break the UI
        self.log.configure(state="normal")
        self.log.insert("end", text)
        # Trim from the top so the widget stays a fixed size. An unbounded Text
        # widget is what made long teleop runs feel laggy.
        excess = int(self.log.index("end-1c").split(".")[0]) - MAX_LOG_LINES
        if excess > 0:
            self.log.delete("1.0", f"{excess + 1}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")


def _quote(part: str) -> str:
    if part == "":
        return '""'
    return f'"{part}"' if " " in part else part


def main() -> None:
    if not ENTRY_SCRIPT.exists():
        raise SystemExit(f"cannot find {ENTRY_SCRIPT}; run this from the repo root")

    root = tk.Tk()
    root.title("RealHand Glove Teleop")
    root.geometry("880x900")
    try:
        ttk.Style().theme_use("vista" if sys.platform == "win32" else "clam")
    except tk.TclError:
        pass

    app = TeleopLauncher(root)

    def on_close() -> None:
        app.stop_process()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
