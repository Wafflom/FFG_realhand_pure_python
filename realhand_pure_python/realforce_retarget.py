"""Pure Python RealForce glove-to-RealHand retargeting."""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from typing import Any, Sequence
import json

try:
    import yaml
except ImportError:  # pragma: no cover - JSON calibration can still be used.
    yaml = None

from .realforce import RealForcePairReader, RealForceSnapshot
from .realhand_core import REALHAND_MODEL_LENGTHS, RealHandCore


THUMB_ABD_CALIBRATION_INDEXES = (1,)


@dataclass
class RealForceCommand:
    """Mapped command arrays for direct hand control."""

    left_positions: list[int] | None
    right_positions: list[int] | None
    left_velocities: list[int] | None
    right_velocities: list[int] | None
    left_raw: list[float] | None = None
    right_raw: list[float] | None = None


class RealForceRetarget:
    """Map RealForce serial glove positions into RealHand motor command lists."""

    def __init__(
        self,
        *,
        left_model: str = "l25",
        right_model: str = "l25",
        left_port: str | None = None,
        right_port: str | None = None,
        baudrate: int | None = None,
        serial_query_hz: float = 60.0,
        calibration_path: str | Path | None = None,
        load_sample_calibration: bool = False,
        left_calibration_side: str = "left",
        right_calibration_side: str = "right",
        left_thumb_abd_calibration_side: str | None = None,
        right_thumb_abd_calibration_side: str | None = None,
        is_debug: bool = False,
        logger=None,
    ) -> None:
        self.left_model = self._normalise_model(left_model)
        self.right_model = self._normalise_model(right_model)
        self.left_calibration_side = self._normalise_calibration_side(left_calibration_side)
        self.right_calibration_side = self._normalise_calibration_side(right_calibration_side)
        self.left_thumb_abd_calibration_side = self._normalise_optional_calibration_side(
            left_thumb_abd_calibration_side
        )
        self.right_thumb_abd_calibration_side = self._normalise_optional_calibration_side(
            right_thumb_abd_calibration_side
        )
        self.handcore = RealHandCore(left_model=self.left_model, right_model=self.right_model)
        self.left_hand = self._build_hand(self.left_model, "left", is_debug=is_debug)
        self.right_hand = self._build_hand(self.right_model, "right", is_debug=is_debug)
        self.reader: RealForcePairReader | None = None

        if left_port or right_port:
            self.reader = RealForcePairReader(
                left_port=left_port,
                right_port=right_port,
                baudrate=baudrate,
                query_hz=serial_query_hz,
                logger=logger,
            )

        default_calibration = Path(__file__).resolve().parent / "config" / "calibration_sample.yml"
        if calibration_path:
            self.load_calibration(calibration_path)
        elif load_sample_calibration and default_calibration.exists():
            self.load_calibration(default_calibration)

    def start(self) -> None:
        if self.reader is None:
            raise RuntimeError("No serial ports were configured for this RealForceRetarget instance.")
        self.reader.start()

    def stop(self) -> None:
        if self.reader is not None:
            self.reader.stop()

    def poll(self) -> RealForceCommand:
        """Read the latest serial snapshots and return mapped command arrays."""
        if self.reader is None:
            raise RuntimeError("No serial ports were configured for this RealForceRetarget instance.")
        left_snapshot, right_snapshot = self.reader.snapshots()
        return self.map_snapshots(left_snapshot=left_snapshot, right_snapshot=right_snapshot)

    def map_snapshots(
        self,
        *,
        left_snapshot: RealForceSnapshot | None = None,
        right_snapshot: RealForceSnapshot | None = None,
    ) -> RealForceCommand:
        left_positions = self._valid_snapshot_positions(left_snapshot)
        right_positions = self._valid_snapshot_positions(right_snapshot)

        if left_snapshot and left_snapshot.version:
            self.left_hand.set_glove_version(left_snapshot.version)
        if right_snapshot and right_snapshot.version:
            self.right_hand.set_glove_version(right_snapshot.version)

        return self.map_glove_positions(left=left_positions, right=right_positions)

    def map_glove_positions(
        self,
        *,
        left: Sequence[float] | None = None,
        right: Sequence[float] | None = None,
    ) -> RealForceCommand:
        """Map 21-value RealForce glove arrays into RealHand motor commands."""
        left_command = None
        right_command = None
        left_velocity = None
        right_velocity = None

        if left is not None:
            left_values = [float(v) for v in left]
            self.left_hand.joint_update(left_values)
            self.left_hand.speed_update()
            left_command = list(self.left_hand.g_jointpositions)
            left_velocity = list(self.left_hand.g_jointvelocity)

        if right is not None:
            right_values = [float(v) for v in right]
            self.right_hand.joint_update(right_values)
            self.right_hand.speed_update()
            right_command = list(self.right_hand.g_jointpositions)
            right_velocity = list(self.right_hand.g_jointvelocity)

        return RealForceCommand(
            left_positions=left_command,
            right_positions=right_command,
            left_velocities=left_velocity,
            right_velocities=right_velocity,
            left_raw=list(left) if left is not None else None,
            right_raw=list(right) if right is not None else None,
        )

    def load_calibration(self, path: str | Path) -> None:
        data = self._load_calibration_file(Path(path))
        self._apply_calibration(
            self.left_hand,
            data,
            side=self.left_calibration_side,
            thumb_abd_side=self.left_thumb_abd_calibration_side,
        )
        self._apply_calibration(
            self.right_hand,
            data,
            side=self.right_calibration_side,
            thumb_abd_side=self.right_thumb_abd_calibration_side,
        )

    def _build_hand(self, model: str, side: str, *, is_debug: bool) -> Any:
        module_name, length = self._hand_module_and_length(model)
        module = import_module(f".realforce_hands.{module_name}", package=__package__)
        class_name = "LeftHand" if side == "left" else "RightHand"
        hand_class = getattr(module, class_name)
        return hand_class(self.handcore, length=length, is_debug=is_debug)

    @staticmethod
    def _hand_module_and_length(model: str) -> tuple[str, int]:
        if model in {"o7", "l7", "o7v1", "o7v3"}:
            return "realforce_l7", 7
        if model == "o6":
            return "realforce_o6", 6
        if model == "l6":
            return "realforce_l6", 6
        if model in {"l25", "g20"}:
            return "realforce_g20", 20
        if model == "l20":
            return "realforce_l20", 20
        if model in {"l10", "l10v7", "l20lite"}:
            return "realforce_l10", 10
        raise ValueError(f"RealForce retargeting does not include a mapper for model '{model}'.")

    @staticmethod
    def _normalise_model(model: str) -> str:
        key = model.lower()
        if key not in REALHAND_MODEL_LENGTHS:
            supported = ", ".join(sorted(REALHAND_MODEL_LENGTHS))
            raise ValueError(f"Unsupported RealHand model '{model}'. Supported models: {supported}")
        if key == "l21":
            raise ValueError("RealForce retargeting does not include an L21 mapper in the original code.")
        return key

    @staticmethod
    def _normalise_calibration_side(side: str) -> str:
        key = side.lower()
        if key in {"left", "l"}:
            return "l"
        if key in {"right", "r"}:
            return "r"
        raise ValueError(f"unsupported calibration side {side!r}; use left or right")

    @classmethod
    def _normalise_optional_calibration_side(cls, side: str | None) -> str | None:
        if side is None:
            return None
        return cls._normalise_calibration_side(side)

    @staticmethod
    def _load_calibration_file(path: Path) -> dict[str, Any]:
        if not path.exists():
            raise FileNotFoundError(f"Calibration file does not exist: {path}")
        if path.suffix.lower() == ".json":
            with path.open("r", encoding="utf-8") as handle:
                return json.load(handle)
        if yaml is None:
            raise ImportError("PyYAML is required to load YAML calibration files: pip install pyyaml")
        with path.open("r", encoding="utf-8") as handle:
            return yaml.safe_load(handle)

    @classmethod
    def _apply_calibration(
        cls,
        hand: Any,
        data: dict[str, Any],
        *,
        side: str,
        thumb_abd_side: str | None = None,
    ) -> None:
        original = data.get(f"jointangleoriginal_{side}")
        fist = data.get(f"jointanglefist_{side}")
        opose = data.get(f"jointangleopose_{side}")
        if thumb_abd_side is not None and thumb_abd_side != side:
            original = cls._merge_calibration_indexes(
                original,
                data.get(f"jointangleoriginal_{thumb_abd_side}"),
                THUMB_ABD_CALIBRATION_INDEXES,
            )
            fist = cls._merge_calibration_indexes(
                fist,
                data.get(f"jointanglefist_{thumb_abd_side}"),
                THUMB_ABD_CALIBRATION_INDEXES,
            )
            opose = cls._merge_calibration_indexes(
                opose,
                data.get(f"jointangleopose_{thumb_abd_side}"),
                THUMB_ABD_CALIBRATION_INDEXES,
            )
        if original is not None:
            hand.calibrationoriginal = list(original)
        if fist is not None:
            hand.calibrationfistpose = list(fist)
        if opose is not None:
            hand.calibrationopose = list(opose)
        if original is not None and fist is not None and opose is not None:
            hand.initialize_mapper()

    @staticmethod
    def _merge_calibration_indexes(
        base: Sequence[float] | None,
        override: Sequence[float] | None,
        indexes: Sequence[int],
    ) -> list[float] | None:
        if base is None or override is None:
            return list(base) if base is not None else None
        merged = list(base)
        for index in indexes:
            if index < len(merged) and index < len(override):
                merged[index] = override[index]
        return merged

    @staticmethod
    def _valid_snapshot_positions(snapshot: RealForceSnapshot | None) -> list[float] | None:
        if snapshot is None:
            return None
        if not snapshot.connected:
            return None
        if snapshot.version is None:
            return None
        if snapshot.position_frame_count <= 0:
            return None
        if not snapshot.positions:
            return None
        if not any(abs(value) > 1e-9 for value in snapshot.positions):
            return None
        return list(snapshot.positions)
