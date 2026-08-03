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


class Tooltip:
    """Hover tooltip for any widget: shows `text` after a short delay."""

    def __init__(self, widget, text: str, delay_ms: int = 450) -> None:
        self.widget = widget
        self.text = text
        self.delay_ms = delay_ms
        self._after_id = None
        self._tip = None
        widget.bind("<Enter>", self._schedule, add="+")
        widget.bind("<Leave>", self._hide, add="+")
        widget.bind("<ButtonPress>", self._hide, add="+")

    def _schedule(self, _event=None) -> None:
        self._cancel()
        self._after_id = self.widget.after(self.delay_ms, self._show)

    def _show(self) -> None:
        if self._tip is not None:
            return
        x = self.widget.winfo_rootx() + 12
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 6
        self._tip = tk.Toplevel(self.widget)
        self._tip.wm_overrideredirect(True)
        self._tip.wm_geometry(f"+{x}+{y}")
        tk.Label(
            self._tip, text=self.text, justify="left", wraplength=420,
            background="#ffffe1", relief="solid", borderwidth=1,
            font=("Segoe UI", 9), padx=8, pady=6,
        ).pack()

    def _cancel(self) -> None:
        if self._after_id is not None:
            self.widget.after_cancel(self._after_id)
            self._after_id = None

    def _hide(self, _event=None) -> None:
        self._cancel()
        if self._tip is not None:
            self._tip.destroy()
            self._tip = None

# Filter tuning passed to the CLI per run - the mapper source is never edited.
#
# Two filters sit between the glove and the motors:
#   input:  a Kalman filter on the raw glove values (realhand_core_ex).
#           Shipped Q=1e-5 measures as a ~170 ms step response that keeps only
#           27% of an 8 Hz hand shake - rapid motion simply vanished.
#           Q=1e-3 responds in ~20 ms and keeps 94% of the same shake while
#           still absorbing sub-unit sensor noise, so that is the default.
#   output: an EMA + per-frame step limit in each hand mapper. Bypassed by
#           default (--no-smoothing): measured glove noise is under one motor
#           unit, so it only added lag (~40 ms at the old 0.7/80 setting).
#
# The "Extra smoothing" checkbox restores both shipped filters for a noisy
# glove: output 0.7/80 plus the stock Kalman.
INPUT_FILTER_Q = 0.001
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

# ---------------------------------------------------------------------------
# Calibration pose illustrations, drawn as canvas vector art so no image files
# are needed. Right hand, palm facing the viewer, y grows downward on a
# 210x215 canvas. Each stroke: points + styling; "accent" marks the digit the
# pose is about, "arrow" draws a motion hint.
_ART_BASE = "#2b4a6b"      # digit outline
_ART_FILL = "#c8d9ea"      # palm fill
_ART_ACCENT = "#c05000"    # the digit this pose is about
_ART_HINT = "#8a8a8a"      # motion arrows

_FINGERS_UP = [                       # index, middle, ring, pinky extended
    [(80, 102), (76, 42)],
    [(101, 100), (100, 30)],
    [(121, 101), (124, 40)],
    [(139, 104), (146, 58)],
]
_PALM = (72, 95, 148, 190)

POSE_ART = {
    "original": {
        "palm": _PALM,
        "strokes": [{"pts": p} for p in _FINGERS_UP]
        + [{"pts": [(78, 152), (54, 124), (44, 102)]}],
    },
    "opose": {
        "palm": _PALM,
        "strokes": [{"pts": p} for p in _FINGERS_UP[1:]]
        + [
            {"pts": [(80, 102), (68, 62), (56, 84)], "accent": True},
            {"pts": [(78, 152), (55, 120), (56, 90)], "accent": True},
        ],
        "ring": (44, 72, 70, 98),     # highlight where thumb and index touch
    },
    "fist": {
        "palm": (68, 92, 152, 192),
        "knuckles": [(68, 80, 90, 102), (90, 76, 112, 98),
                     (112, 78, 134, 100), (132, 84, 152, 104)],
        "strokes": [{"pts": [(72, 148), (112, 162), (132, 156)],
                     "accent": True, "width": 15}],
    },
    "thumb_out": {
        "palm": _PALM,
        "strokes": [{"pts": p} for p in _FINGERS_UP]
        + [{"pts": [(80, 150), (46, 138), (24, 124)], "accent": True}],
        "arrow": [(52, 178), (30, 168), (16, 148)],
    },
    "thumb_in": {
        "palm": _PALM,
        "strokes": [{"pts": p} for p in _FINGERS_UP]
        + [{"pts": [(76, 142), (102, 156), (122, 152)], "accent": True}],
        "arrow": [(34, 120), (52, 146), (74, 158)],
    },
    "fingers_spread": {
        "palm": _PALM,
        "strokes": [
            {"pts": [(80, 102), (58, 48)], "accent": True},
            {"pts": [(101, 100), (94, 30)], "accent": True},
            {"pts": [(121, 101), (138, 40)], "accent": True},
            {"pts": [(139, 104), (160, 64)], "accent": True},
            {"pts": [(78, 152), (54, 124), (44, 102)]},
        ],
        "arrow": [(148, 96), (166, 88), (178, 74)],
    },
    "fingers_together": {
        "palm": _PALM,
        "strokes": [
            {"pts": [(92, 102), (90, 38)], "accent": True},
            {"pts": [(103, 100), (102, 32)], "accent": True},
            {"pts": [(114, 101), (115, 36)], "accent": True},
            {"pts": [(125, 104), (128, 46)], "accent": True},
            {"pts": [(78, 152), (54, 124), (44, 102)]},
        ],
        "arrow": [(162, 70), (148, 72), (138, 72)],
    },
    "finger_roots_bend": {
        "palm": _PALM,
        "strokes": [
            {"pts": [(80, 102), (80, 84), (52, 76)], "accent": True},
            {"pts": [(101, 100), (101, 80), (72, 70)], "accent": True},
            {"pts": [(121, 101), (121, 82), (93, 73)], "accent": True},
            {"pts": [(139, 104), (139, 88), (113, 79)], "accent": True},
            {"pts": [(78, 152), (54, 124), (44, 102)]},
        ],
    },
    "finger_tips_curl": {
        "palm": _PALM,
        "strokes": [
            {"pts": [(80, 102), (76, 52), (62, 56)], "accent": True},
            {"pts": [(101, 100), (99, 36), (84, 40)], "accent": True},
            {"pts": [(121, 101), (123, 44), (108, 48)], "accent": True},
            {"pts": [(139, 104), (144, 64), (130, 68)], "accent": True},
            {"pts": [(78, 152), (54, 124), (44, 102)]},
        ],
    },
    "thumb_rotate": {
        "palm": _PALM,
        "strokes": [{"pts": p} for p in _FINGERS_UP]
        + [{"pts": [(66, 136), (96, 168), (128, 164)], "accent": True}],
        "arrow": [(46, 118), (68, 158), (104, 176)],
    },
    "thumb_tip": {
        "palm": _PALM,
        "strokes": [{"pts": p} for p in _FINGERS_UP]
        + [{"pts": [(78, 152), (56, 126), (46, 106), (58, 94)], "accent": True}],
    },
}


def draw_pose_art(canvas: tk.Canvas, key: str) -> None:
    """Render one calibration pose (or 'done'/'failed') onto the canvas."""
    canvas.delete("all")
    if key == "done":
        canvas.create_line(55, 115, 90, 155, 150, 70, width=16,
                           capstyle="round", joinstyle="round", fill="#1a7f37")
        return
    if key == "failed":
        for pts in ((60, 70, 150, 160), (150, 70, 60, 160)):
            canvas.create_line(*pts, width=14, capstyle="round", fill="#b00020")
        return

    art = POSE_ART.get(key)
    if art is None:
        return
    canvas.create_oval(*art["palm"], fill=_ART_FILL, outline=_ART_BASE, width=3)
    for box in art.get("knuckles", []):
        canvas.create_oval(*box, fill=_ART_FILL, outline=_ART_BASE, width=3)
    for stroke in art["strokes"]:
        flat = [c for pt in stroke["pts"] for c in pt]
        canvas.create_line(
            *flat,
            width=stroke.get("width", 13),
            fill=_ART_ACCENT if stroke.get("accent") else _ART_BASE,
            capstyle="round", joinstyle="round",
            smooth=len(stroke["pts"]) > 2,
        )
    if "ring" in art:
        canvas.create_oval(*art["ring"], outline=_ART_ACCENT, width=3)
    if "arrow" in art:
        flat = [c for pt in art["arrow"] for c in pt]
        canvas.create_line(*flat, width=4, fill=_ART_HINT, smooth=True,
                           arrow=tk.LAST, arrowshape=(14, 16, 7))

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

# The L20 has independent finger-spread, root-vs-tip flexion and extra thumb
# DOFs, so its calibration captures six more poses.
CALIBRATION_POSES_L20 = CALIBRATION_POSES + [
    (
        "fingers_spread",
        "6. Spread",
        "Spread all four fingers apart sideways\n"
        "as far as they comfortably go.",
    ),
    (
        "fingers_together",
        "7. Together",
        "Press your fingers together, straight\n"
        "and touching.",
    ),
    (
        "finger_roots_bend",
        "8. Knuckle Bend",
        "Bend all four fingers 90 degrees at the\n"
        "knuckles, keeping the fingers themselves\n"
        "straight (table-top shape).",
    ),
    (
        "finger_tips_curl",
        "9. Tip Curl",
        "Curl only your fingertips into a claw.\n"
        "Keep the knuckles straight.",
    ),
    (
        "thumb_rotate",
        "10. Thumb Sweep",
        "Sweep your thumb across the palm toward\n"
        "the base of the pinky (opposition).",
    ),
    (
        "thumb_tip",
        "11. Thumb Tip",
        "Curl only the tip of your thumb.\n"
        "Keep everything else straight.",
    ),
]


class CalibrationWindow(tk.Toplevel):
    """Full-screen-ish guided calibration in English.

    Runs the same `--calibrate-realforce` capture as the CLI and translates its
    progress output into large, plain-English pose instructions. The capture
    script itself is untouched; this only reads its stdout.
    """

    def __init__(self, master: tk.Misc, args: list[str], output_path: str,
                 poses: list | None = None) -> None:
        super().__init__(master)
        self.poses = poses if poses is not None else CALIBRATION_POSES
        self.title("Glove Calibration")
        self.geometry("640x810")
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
            text=f"{len(self.poses)} poses will be recorded. "
                 "Match the picture and hold still until it says Done.",
            foreground="#555555",
            wraplength=580,
        ).pack(anchor="w", pady=(2, 12))

        # Step indicators, wrapped into rows of six so the L20 set fits.
        steps = ttk.Frame(outer)
        steps.pack(fill="x", pady=(0, 12))
        self.step_labels: dict[str, ttk.Label] = {}
        for index, (key, title, _desc) in enumerate(self.poses):
            lbl = ttk.Label(
                steps,
                text=title,
                font=("Segoe UI", 10),
                foreground="#999999",
                padding=(8, 4),
            )
            lbl.grid(row=index // 6, column=index % 6, padx=(0, 10), sticky="w")
            self.step_labels[key] = lbl

        # Big instruction card
        card = tk.Frame(outer, bg="#f0f4f8", highlightbackground="#c8d4e0", highlightthickness=1)
        card.pack(fill="both", expand=True, pady=(0, 12))

        self.pose_title = tk.Label(
            card, text="Starting...", font=("Segoe UI", 24, "bold"), bg="#f0f4f8", fg="#1a3a5a"
        )
        self.pose_title.pack(pady=(16, 6))

        self.pose_canvas = tk.Canvas(card, width=210, height=215,
                                     bg="#f0f4f8", highlightthickness=0)
        self.pose_canvas.pack(pady=(0, 4))

        self.pose_desc = tk.Label(
            card,
            text="Connecting to the glove.",
            font=("Segoe UI", 13),
            bg="#f0f4f8",
            fg="#2a2a2a",
            justify="center",
        )
        self.pose_desc.pack(pady=(0, 12))

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
        for key, title, desc in self.poses:
            if f"[calibration] {key}:" in text and self.current_pose != key:
                self.current_pose = key
                self._set_pose_display(title, desc, "Get ready...")
                self._mark_steps(key)
                draw_pose_art(self.pose_canvas, key)
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
            draw_pose_art(self.pose_canvas, "done")
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
        draw_pose_art(self.pose_canvas, "failed")
        for lbl in self.step_labels.values():
            lbl.configure(foreground="#999999", font=("Segoe UI", 10))
        if not self._log_shown:
            self._toggle_log()

    def _mark_steps(self, active_key: str) -> None:
        reached = False
        for key, _title, _desc in self.poses:
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
        # Per-group motion exaggeration multipliers. 1.0 = mapper as calibrated,
        # >1 exaggerates (full travel from less movement), <1 damps. The old 2.0
        # thumb baseline was tuned against a decrepit hand with a near-dead thumb;
        # on healthier hardware it over-drives, so the defaults are gentle now.
        # All three accept values below 1.0 to soften an over-responsive channel.
        self.exagg_thumb_abd = tk.StringVar(value="1.2")
        self.exagg_thumb_flex = tk.StringVar(value="1.0")
        self.exagg_fingers = tk.StringVar(value="0.9")
        # Tunable filter values, prefilled with the measured fast defaults.
        self.input_q = tk.StringVar(value=str(INPUT_FILTER_Q))
        self.output_smoothing = tk.BooleanVar(value=False)
        self.smooth_alpha = tk.StringVar(value=str(SMOOTH_ALPHA))
        self.smooth_step = tk.StringVar(value=str(SMOOTH_MAX_STEP))
        self.query_hz = tk.StringVar(value=str(SERIAL_QUERY_HZ))
        self.send_hz = tk.StringVar(value="100")
        self.continuous = tk.BooleanVar(value=True)
        self.seconds = tk.StringVar(value="30")
        self.verbose_log = tk.BooleanVar(value=False)

        self.status_text = tk.StringVar(value="Idle")

        self._build_ui()
        self._on_form_change()
        self.after(100, self._drain_output)

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

        exagg = ttk.Frame(box)
        exagg.grid(row=5, column=0, columnspan=3, sticky="w")
        lbl_ex = ttk.Label(exagg, text="Exaggeration:")
        lbl_ex.grid(row=0, column=0, sticky="w", **PAD)
        Tooltip(lbl_ex, "How strongly each motion group responds, applied on top of the "
                        "calibration. 1.0 = as calibrated; >1 = the robot exaggerates "
                        "(reaches full travel with less of your movement); <1 = damps it. "
                        "Useful because thumbs vary a lot between people - some need more "
                        "gain before adduction registers, some less.")
        for col, (text, var, tip) in enumerate((
            ("thumb spread", self.exagg_thumb_abd,
             "Thumb adduction/abduction (side swing toward and away from the palm).\n"
             "Raise above 1.0 if the robot thumb barely spreads when yours does;\n"
             "lower below 1.0 if it slams between the extremes."),
            ("thumb squeeze", self.exagg_thumb_flex,
             "Thumb flexion/rotation - the curl and opposition used for squeezing.\n"
             "Covers every thumb channel except the side swing."),
            ("fingers", self.exagg_fingers,
             "All four fingers: flexion (curl) and side-sway channels.\n"
             "Raise if a full fist doesn't fully close the robot hand;\n"
             "lower if the fingers hit their limits too early."),
        )):
            lab = ttk.Label(exagg, text=text)
            lab.grid(row=0, column=1 + col * 2, sticky="e", padx=(10, 2), pady=4)
            ent = ttk.Entry(exagg, textvariable=var, width=6)
            ent.grid(row=0, column=2 + col * 2, sticky="w", pady=4)
            Tooltip(lab, tip)
            Tooltip(ent, tip)


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
            row=0, column=2, rowspan=2, sticky="ns", **PAD
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

        # ---- response tuning ------------------------------------------------
        lbl_q = ttk.Label(box, text="Input filter Q")
        lbl_q.grid(row=4, column=0, sticky="w", **PAD)
        ent_q = ttk.Entry(box, textvariable=self.input_q, width=12)
        ent_q.grid(row=4, column=1, sticky="w", **PAD)
        for w in (lbl_q, ent_q):
            Tooltip(w, "Responsiveness of the Kalman filter on the raw glove values.\n"
                       "Higher = faster and more of your rapid motion survives; "
                       "lower = smoother but laggier.\n\n"
                       "0.00001 (shipped) = 170 ms response, keeps only 27% of an 8 Hz shake\n"
                       "0.001 (default)   =  20 ms response, keeps 94% of an 8 Hz shake\n"
                       "blank             = bypass the input filter entirely (10 ms, fully raw)")

        lbl_gq = ttk.Label(box, text="Glove poll (Hz)")
        lbl_gq.grid(row=4, column=2, sticky="w", **PAD)
        ent_gq = ttk.Entry(box, textvariable=self.query_hz, width=12)
        ent_gq.grid(row=4, column=3, sticky="w", **PAD)
        for w in (lbl_gq, ent_gq):
            Tooltip(w, "How often the glove is asked for a position frame.\n"
                       "The glove firmware tops out near 49 frames/s, reached at 120 Hz; "
                       "the shipped 60 Hz only got 32 frames/s. Values above 120 gain "
                       "nothing, below 120 add sampling latency.")

        chk_out = ttk.Checkbutton(
            box,
            text="Output smoothing",
            variable=self.output_smoothing,
            command=self._on_form_change,
        )
        chk_out.grid(row=5, column=0, sticky="w", **PAD)
        Tooltip(chk_out, "Second filter applied to the final motor commands (EMA + per-frame "
                         "step cap). Off by default: this glove measures under one motor unit "
                         "of noise, so it only added ~40 ms of lag. Turn on for a noisy glove "
                         "or if the hand visibly trembles when yours is still.")

        lbl_a = ttk.Label(box, text="alpha")
        lbl_a.grid(row=5, column=1, sticky="e", **PAD)
        self.alpha_entry = ttk.Entry(box, textvariable=self.smooth_alpha, width=8)
        self.alpha_entry.grid(row=5, column=2, sticky="w", **PAD)
        for w in (lbl_a, self.alpha_entry):
            Tooltip(w, "Output filter: weight of the newest reading (0-1).\n"
                       "1.0 = trust the new value fully (no averaging), lower = heavier "
                       "averaging with older values, smoother but laggier.\n"
                       "Shipped 0.5; 0.7 is a good middle ground.")

        lbl_s = ttk.Label(box, text="max step")
        lbl_s.grid(row=5, column=3, sticky="w", **PAD)
        self.step_entry = ttk.Entry(box, textvariable=self.smooth_step, width=8)
        self.step_entry.grid(row=5, column=3, sticky="e", **PAD)
        for w in (lbl_s, self.step_entry):
            Tooltip(w, "Output filter: hard speed limit - the most a motor command may "
                       "change per frame (0-255 scale).\n"
                       "It caps hand speed directly: 20 (shipped) = full travel ~130 ms at "
                       "100 Hz; 80 = ~30 ms; 255 = uncapped. Lower this first if a joint "
                       "ever oscillates or overshoots.")

        lbl_hz = ttk.Label(box, text="Send rate (Hz)")
        lbl_hz.grid(row=6, column=0, sticky="w", **PAD)
        ent_hz = ttk.Entry(box, textvariable=self.send_hz, width=12)
        ent_hz.grid(row=6, column=1, sticky="w", **PAD)
        for w in (lbl_hz, ent_hz):
            Tooltip(w, "How often mapped commands are sent to the hand over CAN.\n"
                       "100 Hz is comfortable (CAN can do thousands). Lowering adds latency; "
                       "raising past ~100 gains little because the glove only produces ~49 "
                       "fresh readings per second.")

        chk_cont = ttk.Checkbutton(
            box,
            text="Run continuously",
            variable=self.continuous,
            command=self._on_form_change,
        )
        chk_cont.grid(row=6, column=2, sticky="w", **PAD)
        self.seconds_entry = ttk.Entry(box, textvariable=self.seconds, width=12)
        self.seconds_entry.grid(row=6, column=3, sticky="w", **PAD)
        Tooltip(self.seconds_entry, "Run duration in seconds when not running continuously. "
                                    "The run stops by itself after this long.")

        for var in (self.send_hz, self.seconds, self.left_can, self.right_can,
                    self.input_q, self.smooth_alpha, self.smooth_step, self.query_hz,
                    self.exagg_thumb_abd, self.exagg_thumb_flex, self.exagg_fingers):
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
        out_state = "normal" if self.output_smoothing.get() else "disabled"
        self.alpha_entry.configure(state=out_state)
        self.step_entry.configure(state=out_state)
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

    def detect_can(self) -> None:
        """Ask python-can what CAN hardware is actually present."""
        self.append_log("[gui] detecting CAN adapters...\n")
        try:
            import can
        except ImportError as exc:
            self.append_log(f"[gui] python-can not available: {exc}\n")
            return

        try:
            configs = can.detect_available_configs()
        except Exception as exc:
            self.append_log(f"[gui] detection failed: {exc}\n")
            return

        can_configs = [c for c in configs if c.get("interface") != "serial"]
        if not can_configs:
            self.append_log("[gui] no CAN adapters detected.\n")
            self.status_text.set("No CAN adapters")
            return

        for config in can_configs:
            name = config.get("device_name", "")
            self.append_log(
                f"  interface={config.get('interface')} channel={config.get('channel')}"
                + (f" ({name})" if name else "")
                + "\n"
            )

        real = [c for c in can_configs if c.get("interface") != "virtual"]
        if real:
            self.can_type.set(real[0]["interface"])
            self.left_can.set(str(real[0]["channel"]))
            # With a single adapter, point both sides at it so Left-only and
            # Right-only both work; running Both still needs a second adapter
            # and _can_warning() says so.
            self.right_can.set(str(real[1]["channel"] if len(real) > 1 else real[0]["channel"]))
            self.status_text.set(f"Found {len(real)} CAN adapter(s)")
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
            args += ["--serial-query-hz", self.query_hz.get().strip() or str(SERIAL_QUERY_HZ)]
        else:
            args += ["--mcg-host", self.mcg_host.get().strip() or "127.0.0.1"]
            args += ["--mcg-port", self.mcg_port.get().strip() or "9011"]
            if self.mcg_bind_port.get().strip():
                args += ["--mcg-bind-port", self.mcg_bind_port.get().strip()]

        if calibrate:
            # Written by start_calibration() from an explicit Save-as dialog so a
            # capture can never overwrite the calibration file selected above.
            args += ["--calibration-output", self._calibration_output]
            args += ["--calibration-sample-seconds", "3", "--calibration-ready-seconds", "3"]
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

        q = self.input_q.get().strip()
        if q:
            args += ["--input-filter-q", q]
        else:
            args.append("--no-input-filter")     # blank Q = fully raw input
        if self.output_smoothing.get():
            args += ["--smooth-alpha", self.smooth_alpha.get().strip() or str(SMOOTH_ALPHA),
                     "--smooth-max-step", self.smooth_step.get().strip() or str(SMOOTH_MAX_STEP)]
        else:
            args.append("--no-smoothing")
        args.append("--reverse-thumb-abduction" if self.reverse_thumb.get()
                    else "--no-reverse-thumb-abduction")
        for flag, var in (("--exaggerate-thumb-abd", self.exagg_thumb_abd),
                          ("--exaggerate-thumb-flex", self.exagg_thumb_flex),
                          ("--exaggerate-fingers", self.exagg_fingers)):
            value = var.get().strip()
            if value and value not in ("1", "1.0"):
                args += [flag, value]
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
    def scan_ports(self) -> None:
        self.append_log("[gui] scanning serial ports...\n")
        try:
            result = subprocess.run(
                [sys.executable, "-u", str(ENTRY_SCRIPT), "--scan"],
                cwd=str(REPO_ROOT),
                capture_output=True,
                text=True,
                timeout=30,
            )
        except Exception as exc:
            self.append_log(f"[gui] scan failed: {exc}\n")
            return

        self.append_log(result.stdout or "")
        if result.stderr:
            self.append_log(result.stderr)

        ports = [
            line.strip()
            for line in (result.stdout or "").splitlines()[1:]
            if line.strip()
        ]
        self.left_port_combo.configure(values=ports)
        self.right_port_combo.configure(values=ports)
        if ports:
            if not self.left_port.get():
                self.left_port.set(ports[0])
            if not self.right_port.get() and len(ports) > 1:
                self.right_port.set(ports[1])
            self.status_text.set(f"Found {len(ports)} port(s)")
        else:
            self.status_text.set("No ports found")

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

        q = self.input_q.get().strip()
        if q:
            try:
                if float(q) <= 0:
                    return "Input filter Q must be greater than 0 (or blank to bypass)."
            except ValueError:
                return f"Input filter Q {q!r} is not a number. Use e.g. 0.001, or blank to bypass."
        if self.output_smoothing.get():
            try:
                alpha = float(self.smooth_alpha.get().strip())
                if not 0.0 < alpha <= 1.0:
                    return "Output alpha must be between 0 and 1."
            except ValueError:
                return f"Output alpha {self.smooth_alpha.get()!r} is not a number."
            try:
                step = float(self.smooth_step.get().strip())
                if not 1 <= step <= 255:
                    return "Output max step must be between 1 and 255."
            except ValueError:
                return f"Output max step {self.smooth_step.get()!r} is not a number."
        try:
            if float(self.query_hz.get().strip() or "0") <= 0:
                return "Glove poll rate must be greater than 0 Hz."
        except ValueError:
            return f"Glove poll rate {self.query_hz.get()!r} is not a number."

        for name, var in (("thumb spread", self.exagg_thumb_abd),
                          ("thumb squeeze", self.exagg_thumb_flex),
                          ("fingers", self.exagg_fingers)):
            value = var.get().strip()
            if not value:
                continue
            try:
                # Values below 1.0 damp a channel, above 1.0 exaggerate it.
                if not 0.05 <= float(value) <= 10:
                    return f"Exaggeration ({name}) should be between 0.05 and 10; 1.0 is neutral."
            except ValueError:
                return f"Exaggeration ({name}) {value!r} is not a number."

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
        poses = CALIBRATION_POSES_L20 if model == "l20" else CALIBRATION_POSES
        CalibrationWindow(self.winfo_toplevel(), self.build_command(calibrate=True),
                          output, poses=poses)
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
