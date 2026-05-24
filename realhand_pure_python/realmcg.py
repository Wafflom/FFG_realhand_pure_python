"""Pure Python RealMCG UDP/serial reader and command mapper.

RealMCG data is received as JSON.  The documented public path is UDP broadcast
from the Linker MCG upper-computer software; serial mode is best-effort for USB
receivers that expose the same JSON stream directly.  The mapper converts the
25 channel hand payload into per-hand command positions and velocities without
ROS.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Sequence

try:
    import serial
except ImportError:  # pragma: no cover - serial mode requires pyserial.
    serial = None


NODES_PER_HAND = 25
MCG_ARRAY_ORDER = ("pitch", "side", "roll", "two_pitch", "end_pitch")


@dataclass(frozen=True)
class RealMCGFrame:
    frame_index: int
    left_raw: List[float]
    right_raw: List[float]
    timestamp: float


@dataclass(frozen=True)
class RealHandCommand:
    left_positions: List[float]
    right_positions: List[float]
    left_velocities: List[float]
    right_velocities: List[float]
    left_raw: List[float]
    right_raw: List[float]
    frame_index: int
    timestamp: float


class RealMCGUdpClient:
    """Receive RealMCG UDP JSON frames.

    Args:
        host: Target host to send the initial CONNECT packet to.
        port: Target port used by the RealMCG sender/server.
        bind_address: Optional local bind tuple.  Use this when the sender
            pushes data to a fixed local UDP port.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 9011,
        bind_address: Optional[tuple[str, int]] = None,
        buffer_size: int = 2048,
        logger: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.target_address = (host, int(port))
        self.bind_address = bind_address
        self.buffer_size = buffer_size
        self.logger = logger

        self.socket_udp: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._frame_index = 0
        self._latest = RealMCGFrame(0, [0.0] * NODES_PER_HAND, [0.0] * NODES_PER_HAND, time.time())
        self.on_frame: Optional[Callable[[RealMCGFrame], None]] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self.socket_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.bind_address is not None:
            self.socket_udp.bind(self.bind_address)
        self.socket_udp.settimeout(1.0)
        self.socket_udp.sendto(json.dumps({"action": "CONNECT"}).encode("utf-8"), self.target_address)
        self._running.set()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        if self.socket_udp:
            self.socket_udp.close()
            self.socket_udp = None

    close = stop

    def latest_frame(self) -> RealMCGFrame:
        with self._lock:
            return RealMCGFrame(
                frame_index=self._latest.frame_index,
                left_raw=self._latest.left_raw.copy(),
                right_raw=self._latest.right_raw.copy(),
                timestamp=self._latest.timestamp,
            )

    def send(self, data: bytes) -> None:
        if not self.socket_udp:
            raise RuntimeError("UDP client is not started")
        self.socket_udp.sendto(data, self.target_address)

    def _recv_loop(self) -> None:
        while self._running.is_set():
            try:
                assert self.socket_udp is not None
                payload, _addr = self.socket_udp.recvfrom(self.buffer_size)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                frame = self._parse_payload(payload)
            except Exception as exc:
                self._log(f"failed to parse RealMCG payload: {exc}")
                continue

            with self._lock:
                self._latest = frame
            if self.on_frame:
                self.on_frame(frame)

    def _parse_payload(self, payload: bytes) -> RealMCGFrame:
        data = json.loads(payload.decode("utf-8", errors="replace"))
        self._frame_index += 1
        return frame_from_mcg_json(data, self._frame_index, self.latest_frame())

    def _log(self, message: str) -> None:
        if self.logger:
            self.logger(message)


class RealMCGSerialClient:
    """Receive RealMCG JSON frames from a USB serial receiver."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.02,
        logger: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = timeout
        self.logger = logger

        self.serial_port = None
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._frame_index = 0
        self._latest = RealMCGFrame(0, [0.0] * NODES_PER_HAND, [0.0] * NODES_PER_HAND, time.time())
        self._text_buffer = ""
        self.on_frame: Optional[Callable[[RealMCGFrame], None]] = None

    def start(self) -> None:
        _require_pyserial()
        if self._thread and self._thread.is_alive():
            return
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self._running.set()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None

    close = stop

    def latest_frame(self) -> RealMCGFrame:
        with self._lock:
            return RealMCGFrame(
                frame_index=self._latest.frame_index,
                left_raw=self._latest.left_raw.copy(),
                right_raw=self._latest.right_raw.copy(),
                timestamp=self._latest.timestamp,
            )

    def _recv_loop(self) -> None:
        while self._running.is_set():
            try:
                if not self.serial_port:
                    break
                waiting = self.serial_port.in_waiting
                chunk = self.serial_port.read(waiting or 1)
                if not chunk:
                    continue
                self._text_buffer += chunk.decode("utf-8", errors="ignore")
                for data in extract_json_objects(self._text_buffer):
                    self._frame_index += 1
                    frame = frame_from_mcg_json(data, self._frame_index, self.latest_frame())
                    with self._lock:
                        self._latest = frame
                    if self.on_frame:
                        self.on_frame(frame)
                self._text_buffer = trim_consumed_json_text(self._text_buffer)
            except Exception as exc:  # pragma: no cover - hardware path
                self._log(f"serial MCG read error: {exc}")
                time.sleep(0.02)

    def _log(self, message: str) -> None:
        if self.logger:
            self.logger(message)


class RealMCGMapper:
    """Map RealMCG 25-channel payloads to RealHand command positions."""

    MODEL_LENGTHS = {
        "o6": 6,
        "l6": 6,
        "o7": 7,
        "o7v1": 7,
        "o7v3": 7,
        "l7": 7,
        "l10": 10,
        "l10v7": 10,
        "l20": 20,
        "l20lite": 10,
        "l21": 25,
        "l25": 20,
        "g20": 20,
    }

    def __init__(self, left_model: str = "l25", right_model: str = "l25") -> None:
        self.left_model = normalize_model(left_model)
        self.right_model = normalize_model(right_model)
        self._left_speed = _VelocityState(self.MODEL_LENGTHS[self.left_model])
        self._right_speed = _VelocityState(self.MODEL_LENGTHS[self.right_model])

    def map_frame(self, frame: RealMCGFrame) -> RealHandCommand:
        left_positions = map_mcg_values(frame.left_raw, self.left_model)
        right_positions = map_mcg_values(frame.right_raw, self.right_model)
        left_velocities = self._left_speed.update(left_positions)
        right_velocities = self._right_speed.update(right_positions)
        return RealHandCommand(
            left_positions=left_positions,
            right_positions=right_positions,
            left_velocities=left_velocities,
            right_velocities=right_velocities,
            left_raw=frame.left_raw.copy(),
            right_raw=frame.right_raw.copy(),
            frame_index=frame.frame_index,
            timestamp=frame.timestamp,
        )


class RealMCGRetarget:
    """Wire RealMCGUdpClient to RealMCGMapper and emit command callbacks."""

    def __init__(
        self,
        host: str,
        port: int,
        left_model: str = "l25",
        right_model: str = "l25",
        bind_address: Optional[tuple[str, int]] = None,
        serial_port: Optional[str] = None,
        serial_baudrate: int = 115200,
        on_command: Optional[Callable[[RealHandCommand], None]] = None,
        logger: Optional[Callable[[str], None]] = None,
    ) -> None:
        if serial_port:
            self.client = RealMCGSerialClient(serial_port, baudrate=serial_baudrate, logger=logger)
        else:
            self.client = RealMCGUdpClient(host, port, bind_address=bind_address, logger=logger)
        self.mapper = RealMCGMapper(left_model=left_model, right_model=right_model)
        self.on_command = on_command
        self.latest_command: Optional[RealHandCommand] = None
        self.client.on_frame = self._handle_frame

    def start(self) -> None:
        self.client.start()

    def stop(self) -> None:
        self.client.stop()

    close = stop

    def _handle_frame(self, frame: RealMCGFrame) -> None:
        command = self.mapper.map_frame(frame)
        self.latest_command = command
        if self.on_command:
            self.on_command(command)


def extract_mcg_hand_values(hand_data: Dict) -> List[float]:
    values: List[float] = []
    for key in MCG_ARRAY_ORDER:
        source = hand_data.get(key, [])
        chunk = [float(item) for item in list(source)[:5]]
        if len(chunk) < 5:
            chunk.extend([0.0] * (5 - len(chunk)))
        values.extend(chunk)
    if len(values) < NODES_PER_HAND:
        values.extend([0.0] * (NODES_PER_HAND - len(values)))
    return values[:NODES_PER_HAND]


def frame_from_mcg_json(data: Dict, frame_index: int, current: RealMCGFrame) -> RealMCGFrame:
    left = current.left_raw
    right = current.right_raw
    if "leftHand" in data:
        left = extract_mcg_hand_values(data["leftHand"])
    if "rightHand" in data:
        right = extract_mcg_hand_values(data["rightHand"])
    return RealMCGFrame(frame_index, left, right, time.time())


def extract_json_objects(text: str) -> List[Dict]:
    objects = []
    for raw in _complete_json_strings(text):
        try:
            value = json.loads(raw)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            objects.append(value)
    return objects


def trim_consumed_json_text(text: str) -> str:
    end = 0
    for _start, stop in _complete_json_spans(text):
        end = stop
    return text[end:][-8192:]


def _complete_json_strings(text: str) -> List[str]:
    return [text[start:stop] for start, stop in _complete_json_spans(text)]


def _complete_json_spans(text: str) -> List[tuple[int, int]]:
    spans = []
    depth = 0
    start = None
    in_string = False
    escape = False
    for index, char in enumerate(text):
        if in_string:
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
            continue
        if char == "{":
            if depth == 0:
                start = index
            depth += 1
        elif char == "}" and depth:
            depth -= 1
            if depth == 0 and start is not None:
                spans.append((start, index + 1))
                start = None
    return spans


def map_mcg_values(values: Sequence[float], model: str) -> List[float]:
    model = normalize_model(model)
    raw = list(values)
    if len(raw) < NODES_PER_HAND:
        raw.extend([0.0] * (NODES_PER_HAND - len(raw)))

    if model in ("o6", "l6"):
        return [raw[0], raw[5], raw[1], raw[2], raw[3], raw[4]]
    if model in ("o7", "o7v1", "o7v3", "l7"):
        return [raw[0], raw[5], raw[1], raw[2], raw[3], raw[4], raw[10]]
    if model in ("l10", "l10v7", "l20lite"):
        return [raw[0], raw[5], raw[1], raw[2], raw[3], raw[4], raw[6], raw[8], raw[9], raw[10]]
    if model in ("l20", "l25", "g20"):
        return raw[:20]
    if model == "l21":
        return raw[:25]
    raise ValueError(f"unsupported RealHand model: {model}")


def normalize_model(model: str) -> str:
    model = str(model).strip().lower()
    if model not in RealMCGMapper.MODEL_LENGTHS:
        raise ValueError(f"unsupported RealHand model: {model}")
    return model


def _require_pyserial() -> None:
    if serial is None:
        raise RuntimeError("pyserial is required for MCG serial mode: pip install pyserial")


class _VelocityState:
    def __init__(self, length: int) -> None:
        self.last_positions = [255.0] * length

    def update(self, positions: Sequence[float]) -> List[float]:
        # The original MCG path ultimately uses 255 as the outgoing velocity for
        # most supported hands.  Keep that behavior here so the pure Python
        # command has the same shape without ROS.
        self.last_positions = list(positions)
        return [255.0] * len(positions)
