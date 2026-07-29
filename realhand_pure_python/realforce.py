"""Pure Python RealForce USB serial reader.

The reader speaks the force-glove frame protocol directly through pyserial and
keeps the latest glove angles/forces in normal Python lists.  It does not
publish ROS messages or import ROS modules.
"""

from __future__ import annotations

import enum
import glob
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Sequence

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # pragma: no cover - depends on user environment
    serial = None


FRAME_HEADER = 0x5D
MAX_FRAME_DATA_SIZE = 255
DEFAULT_BAUDRATES = [2_000_000, 460_800, 1_000_000, 921_600]


class HandSide(str, enum.Enum):
    LEFT = "left"
    RIGHT = "right"


class CommandCode(enum.IntEnum):
    VERSION_QUERY = 0x01
    SET_FLAG = 0x02
    POSITION_QUERY = 0x03
    FORCE_FEEDBACK = 0x04
    A3_POSITION = 0xA3
    A6_POSITION = 0xA6
    A7_FORCE = 0xA7


@dataclass(frozen=True)
class RealForceSnapshot:
    side: Optional[HandSide]
    version: Optional[str]
    positions: List[float]
    commanded_forces: List[float]
    measured_forces: List[int]
    connected: bool
    frame_count: int
    position_frame_count: int
    last_rx_time: float
    last_position_time: float


class _FrameParser:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.state = "header"
        self.buf = bytearray()
        self.expected_len = 0
        self.checksum = 0

    def process(self, byte: int) -> Optional[bytes]:
        byte &= 0xFF

        if self.state == "header":
            if byte == FRAME_HEADER:
                self.buf = bytearray([byte])
                self.checksum = byte
                self.state = "cmd"
            return None

        if self.state == "cmd":
            self.buf.append(byte)
            self.checksum = (self.checksum + byte) & 0xFF
            self.state = "length"
            return None

        if self.state == "length":
            self.buf.append(byte)
            self.checksum = (self.checksum + byte) & 0xFF
            self.expected_len = 3 + byte + 1
            if byte > MAX_FRAME_DATA_SIZE:
                self.reset()
            elif byte == 0:
                self.state = "checksum"
            else:
                self.state = "data"
            return None

        if self.state == "data":
            self.buf.append(byte)
            self.checksum = (self.checksum + byte) & 0xFF
            if len(self.buf) >= self.expected_len - 1:
                self.state = "checksum"
            return None

        if self.state == "checksum":
            if self.checksum == byte:
                self.buf.append(byte)
                frame = bytes(self.buf)
                self.reset()
                return frame
            self.reset()
            return None

        self.reset()
        return None


class RealForceSerialReader:
    """Read one RealForce glove over USB serial.

    Args:
        expected_side: Optional side check.  The device-reported side is still
            exposed even when this is unset.
        baudrates: Baudrates used by scan helpers.
        logger: Optional callback receiving log text.
    """

    def __init__(
        self,
        expected_side: Optional[HandSide | str] = None,
        baudrates: Optional[Sequence[int]] = None,
        query_interval_s: float = 1 / 60,
        logger: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.expected_side = _coerce_side(expected_side)
        self.baudrates = list(baudrates or DEFAULT_BAUDRATES)
        self.query_interval_s = max(0.001, float(query_interval_s))
        self.logger = logger

        self.serial_port = None
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._last_rx = 0.0
        self._last_position_time = 0.0
        self._frame_count = 0
        self._position_frame_count = 0

        self.detected_side: Optional[HandSide] = None
        self.version: Optional[str] = None
        self.positions: List[float] = [0.0] * 21
        self.commanded_forces: List[float] = [0.0] * 5
        self.measured_forces: List[int] = [0] * 5

    @property
    def connected(self) -> bool:
        return bool(self.serial_port and self.serial_port.is_open)

    def snapshot(self) -> RealForceSnapshot:
        with self._lock:
            return RealForceSnapshot(
                side=self.detected_side,
                version=self.version,
                positions=self.positions.copy(),
                commanded_forces=self.commanded_forces.copy(),
                measured_forces=self.measured_forces.copy(),
                connected=self.connected,
                frame_count=self._frame_count,
                position_frame_count=self._position_frame_count,
                last_rx_time=self._last_rx,
                last_position_time=self._last_position_time,
            )

    def open(self, port: str, baudrate: int) -> bool:
        _require_pyserial()
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.001,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
            )
            self._last_rx = time.time()
            return True
        except serial.SerialException as exc:
            self._log(f"failed to open {port}: {exc}")
            self.serial_port = None
            return False

    def start(self) -> None:
        if not self.connected:
            raise RuntimeError("serial port is not open")
        if self._thread and self._thread.is_alive():
            return
        self._running.set()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.send_version_query()

    def stop(self) -> None:
        self._running.clear()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    close = stop

    def send_version_query(self) -> None:
        self._write(pack_frame(CommandCode.VERSION_QUERY))

    def send_position_query(self) -> None:
        self._write(pack_frame(CommandCode.POSITION_QUERY))

    def send_force_feedback(self, forces: Sequence[float]) -> None:
        values = [float(v) for v in forces[:5]]
        if len(values) < 5:
            values.extend([0.0] * (5 - len(values)))
        with self._lock:
            self.commanded_forces = values
        payload = struct.pack(f"{len(values)}f", *values)
        self._write(pack_frame(CommandCode.FORCE_FEEDBACK, payload))

    def open_first_available(
        self,
        ports: Optional[Iterable[str]] = None,
        timeout: float = 0.8,
    ) -> Optional[tuple[str, int]]:
        """Scan ports and open the first device that replies to version query."""

        for port in list(ports or scan_usb_serial_ports()):
            for baudrate in self.baudrates:
                if not self.open(port, baudrate):
                    continue
                self.start()
                deadline = time.time() + timeout
                while time.time() < deadline:
                    if self.detected_side is not None:
                        if self.expected_side and self.detected_side != self.expected_side:
                            break
                        return port, baudrate
                    self.send_version_query()
                    time.sleep(0.1)
                self.stop()
        return None

    def _read_loop(self) -> None:
        parser = _FrameParser()
        next_query_time = 0.0
        while self._running.is_set():
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    self._last_rx = time.time()
                    for byte in data:
                        frame = parser.process(byte)
                        if frame:
                            self._handle_frame(frame)

                now = time.monotonic()
                if now >= next_query_time:
                    if self.detected_side is None:
                        self.send_version_query()
                    else:
                        self.send_position_query()
                    next_query_time = now + self.query_interval_s

                time.sleep(0.001)
            except Exception as exc:  # pragma: no cover - hardware path
                self._log(f"serial read error: {exc}")
                time.sleep(0.02)

    def _handle_frame(self, frame: bytes) -> None:
        cmd = frame[1]
        length = frame[2]
        payload = frame[3 : 3 + length]
        now = time.time()

        with self._lock:
            self._frame_count += 1
            self._last_rx = now

        if cmd == CommandCode.VERSION_QUERY:
            if len(payload) < 5:
                return
            value = struct.unpack("<I", payload[:4])[0]
            value_str = f"{value:05d}"
            version = f"{int(value_str[0])}.{int(value_str[1:3])}.{int(value_str[3:5])}"
            side = HandSide.LEFT if payload[4] == 0 else HandSide.RIGHT if payload[4] == 1 else None
            with self._lock:
                self.version = version
                self.detected_side = side
            return

        if cmd in (CommandCode.POSITION_QUERY, CommandCode.A3_POSITION):
            if len(payload) % 4:
                return
            values = [struct.unpack("<f", payload[i : i + 4])[0] * 0.017453292519943295 for i in range(0, len(payload), 4)]
            with self._lock:
                self.positions = values
                self._position_frame_count += 1
                self._last_position_time = now
            return

        if cmd == CommandCode.FORCE_FEEDBACK:
            if len(payload) % 2:
                return
            values = [struct.unpack(">h", payload[i : i + 2])[0] for i in range(0, len(payload), 2)]
            with self._lock:
                self.measured_forces = values
            return

        if cmd == CommandCode.A6_POSITION:
            if len(payload) % 2:
                return
            values = [
                struct.unpack("<h", payload[i : i + 2])[0] / 100.0 * 0.017453292519943295
                for i in range(0, len(payload), 2)
            ]
            with self._lock:
                self.positions = values
                self._position_frame_count += 1
                self._last_position_time = now
            self._write(pack_frame(CommandCode.A7_FORCE, struct.pack(f"{len(self.commanded_forces)}f", *self.commanded_forces)))

    def _write(self, data: bytes) -> None:
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(data)

    def _log(self, message: str) -> None:
        if self.logger:
            self.logger(message)


class RealForcePairReader:
    """Manage left/right RealForce readers and emit plain snapshots."""

    def __init__(
        self,
        left_port: Optional[str] = None,
        right_port: Optional[str] = None,
        baudrate: Optional[int] = None,
        query_hz: float = 60.0,
        logger: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.logger = logger
        self.baudrates = [baudrate] if baudrate is not None else DEFAULT_BAUDRATES
        self.query_interval_s = 1.0 / query_hz if query_hz > 0 else 1 / 60
        self.left = RealForceSerialReader(
            HandSide.LEFT,
            baudrates=self.baudrates,
            query_interval_s=self.query_interval_s,
            logger=logger,
        )
        self.right = RealForceSerialReader(
            HandSide.RIGHT,
            baudrates=self.baudrates,
            query_interval_s=self.query_interval_s,
            logger=logger,
        )
        self.left_port = left_port
        self.right_port = right_port
        self.baudrate = baudrate
        self.left_opened: Optional[tuple[str, int]] = None
        self.right_opened: Optional[tuple[str, int]] = None

    def start(self) -> None:
        explicit_ports = [port for port in (self.left_port, self.right_port) if port]
        ports = _unique_ports(explicit_ports) if explicit_ports else scan_usb_serial_ports()
        self._open_ports_by_reported_side(ports)

    def stop(self) -> None:
        self.left.stop()
        self.right.stop()

    def snapshots(self) -> tuple[RealForceSnapshot, RealForceSnapshot]:
        return self.left.snapshot(), self.right.snapshot()

    def _open_ports_by_reported_side(self, ports: Iterable[str]) -> None:
        self.left_opened = None
        self.right_opened = None

        for port in ports:
            if self.left_opened and self.right_opened:
                break

            reader = RealForceSerialReader(
                expected_side=None,
                baudrates=self.baudrates,
                query_interval_s=self.query_interval_s,
                logger=self.logger,
            )
            opened = reader.open_first_available(ports=[port])
            if not opened:
                continue

            side = reader.snapshot().side
            if side == HandSide.LEFT and self.left_opened is None:
                self.left.stop()
                self.left = reader
                self.left_opened = opened
                self._log(f"assigned {opened[0]} at {opened[1]} baud to LEFT RealForce glove")
            elif side == HandSide.RIGHT and self.right_opened is None:
                self.right.stop()
                self.right = reader
                self.right_opened = opened
                self._log(f"assigned {opened[0]} at {opened[1]} baud to RIGHT RealForce glove")
            else:
                self._log(f"ignoring {opened[0]}: reported side={side}")
                reader.stop()

    def _log(self, message: str) -> None:
        if self.logger:
            self.logger(message)


def pack_frame(cmd: int, payload: bytes = b"") -> bytes:
    header = struct.pack("BBB", FRAME_HEADER, int(cmd), len(payload))
    checksum = sum(header + payload) & 0xFF
    return header + payload + struct.pack("B", checksum)


def scan_usb_serial_ports() -> List[str]:
    _require_pyserial()
    devices = []
    for info in serial.tools.list_ports.comports():
        name = info.device
        description = (info.description or "").lower()
        hwid = (info.hwid or "").upper()
        if name.startswith(("/dev/ttyUSB", "/dev/ttyACM", "/dev/ttyXRUSB", "/dev/ttyOBC")):
            devices.append(name)
        elif "usb" in description or "USB" in hwid:
            devices.append(name)
    # Direct device-node sweep. comports() can come up short when udev metadata
    # is missing, and some distros ship a vendor ch34x driver that names the
    # glove /dev/ttyCH341USB* instead of /dev/ttyUSB*.
    for pattern in (
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/ttyXRUSB*",
        "/dev/ttyOBC*",
        "/dev/ttyCH341USB*",
        "/dev/ttyCH34x*",
    ):
        for name in glob.glob(pattern):
            if name not in devices:
                devices.append(name)
    return sorted(devices)


def _unique_ports(ports: Iterable[str]) -> list[str]:
    result = []
    seen = set()
    for port in ports:
        if port in seen:
            continue
        seen.add(port)
        result.append(port)
    return result


def _coerce_side(value: Optional[HandSide | str]) -> Optional[HandSide]:
    if value is None or isinstance(value, HandSide):
        return value
    text = str(value).lower()
    if text in ("left", "l"):
        return HandSide.LEFT
    if text in ("right", "r"):
        return HandSide.RIGHT
    raise ValueError(f"unknown hand side: {value}")


def _require_pyserial() -> None:
    if serial is None:
        raise RuntimeError("pyserial is required: pip install pyserial")
