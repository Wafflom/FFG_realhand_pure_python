"""Pure Python RealHand joint-limit and motor-command mapping helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:  # pragma: no cover - exercised only in incomplete envs.
    yaml = None


REALHAND_MODEL_LENGTHS: dict[str, int] = {
    "o7": 7,
    "l7": 7,
    "o6": 6,
    "l6": 6,
    "o7v1": 7,
    "o7v3": 7,
    "l10": 10,
    "l10v7": 10,
    "l20": 20,
    "l20lite": 10,
    "l25": 20,
    "g20": 20,
    "l21": 25,
}


_URDF_MODEL_ALIASES = {
    "l20lite": "l10",
}


class RealHandCore:
    """Minimal replacement for the ROS package HandCore used by RealForce maps."""

    def __init__(
        self,
        right_model: str = "l25",
        left_model: str = "l25",
        config_path: str | Path | None = None,
        assets_dir: str | Path | None = None,
    ) -> None:
        self.package_dir = Path(__file__).resolve().parent
        self.robot_name_str_r = self._normalise_model(right_model)
        self.robot_name_str_l = self._normalise_model(left_model)
        self.config_path = Path(config_path) if config_path else self.package_dir / "config" / "hand_config.yml"
        self.assets_dir = (
            Path(assets_dir)
            if assets_dir
            else self.package_dir / "assets" / "robots" / "hands" / "real_hand"
        )

        handconfig = self._read_yaml(self.config_path)
        right_config_model = self._config_model(self.robot_name_str_r)
        left_config_model = self._config_model(self.robot_name_str_l)

        self.righturdfpath = self._urdf_path(self.robot_name_str_r, "right")
        self.lefturdfpath = self._urdf_path(self.robot_name_str_l, "left")

        self.dataminvalue_r = self._change_list(handconfig[f"commandlower_right_{right_config_model}"])
        self.datamaxvalue_r = self._change_list(handconfig[f"commandupper_right_{right_config_model}"])
        self.sourcedataindex_r = self._change_list(handconfig[f"commandsourcedataindex_right_{right_config_model}"])
        self.urdfdataindex_r = self._change_list(handconfig[f"urdfdataindex_right_{right_config_model}"])

        self.dataminvalue_l = self._change_list(handconfig[f"commandlower_left_{left_config_model}"])
        self.datamaxvalue_l = self._change_list(handconfig[f"commandupper_left_{left_config_model}"])
        self.sourcedataindex_l = self._change_list(handconfig[f"commandsourcedataindex_left_{left_config_model}"])
        self.urdfdataindex_l = self._change_list(handconfig[f"urdfdataindex_left_{left_config_model}"])

        self.hand_lower_limits_r, self.hand_upper_limits_r, self.hand_joint_ranges_r = self.get_joint_limits(
            self.righturdfpath
        )
        self.hand_lower_limits_l, self.hand_upper_limits_l, self.hand_joint_ranges_l = self.get_joint_limits(
            self.lefturdfpath
        )

        self.hand_numjoints_r = REALHAND_MODEL_LENGTHS[self.robot_name_str_r]
        self.hand_numjoints_l = REALHAND_MODEL_LENGTHS[self.robot_name_str_l]

    def trans_to_motor_left(self, temp_l: list[float]) -> list[int]:
        return self._trans_to_motor(
            temp_l,
            num_joints=self.hand_numjoints_l,
            source_indexes=self.sourcedataindex_l,
            urdf_indexes=self.urdfdataindex_l,
            command_lower=self.dataminvalue_l,
            command_upper=self.datamaxvalue_l,
            joint_lower=self.hand_lower_limits_l,
            joint_upper=self.hand_upper_limits_l,
        )

    def trans_to_motor_right(self, temp_r: list[float]) -> list[int]:
        return self._trans_to_motor(
            temp_r,
            num_joints=self.hand_numjoints_r,
            source_indexes=self.sourcedataindex_r,
            urdf_indexes=self.urdfdataindex_r,
            command_lower=self.dataminvalue_r,
            command_upper=self.datamaxvalue_r,
            joint_lower=self.hand_lower_limits_r,
            joint_upper=self.hand_upper_limits_r,
        )

    @staticmethod
    def get_joint_limits(urdf_path: str | Path) -> tuple[list[float], list[float], list[float]]:
        path = Path(urdf_path)
        if not path.exists():
            raise FileNotFoundError(f"URDF path does not exist: {path}")

        root = ET.parse(path).getroot()
        joint_lower_limits: list[float] = []
        joint_upper_limits: list[float] = []
        joint_ranges: list[float] = []

        for joint in root.findall("joint"):
            joint_type = joint.attrib.get("type", "")
            if joint_type == "fixed":
                continue

            limit = joint.find("limit")
            if limit is not None and "lower" in limit.attrib and "upper" in limit.attrib:
                lower = float(limit.attrib["lower"])
                upper = float(limit.attrib["upper"])
            elif joint_type in {"continuous", "revolute"}:
                lower = -3.1415926535
                upper = 3.1415926535
            elif joint_type == "prismatic":
                lower = -1.0
                upper = 1.0
            else:
                lower = float("-inf")
                upper = float("inf")

            joint_lower_limits.append(lower)
            joint_upper_limits.append(upper)
            joint_ranges.append(upper - lower)

        return joint_lower_limits, joint_upper_limits, joint_ranges

    def _trans_to_motor(
        self,
        source_values: list[float],
        *,
        num_joints: int,
        source_indexes: list[int | None],
        urdf_indexes: list[int | None],
        command_lower: list[float | None],
        command_upper: list[float | None],
        joint_lower: list[float],
        joint_upper: list[float],
    ) -> list[int]:
        jointpositions = [255.0] * num_joints
        for i in range(num_joints):
            source_index = source_indexes[i]
            urdf_index = urdf_indexes[i]
            lower_cmd = command_lower[i]
            upper_cmd = command_upper[i]
            if source_index is None or urdf_index is None or lower_cmd is None or upper_cmd is None:
                continue
            if source_index >= len(source_values):
                continue
            if urdf_index >= len(joint_lower):
                continue

            value = self._clamp(source_values[source_index], joint_lower[urdf_index], joint_upper[urdf_index])
            mapped = self._scale_value(value, joint_lower[urdf_index], joint_upper[urdf_index], lower_cmd, upper_cmd)
            jointpositions[i] = int(self._clamp(mapped, 0, 255))
        return [int(v) for v in jointpositions]

    def _urdf_path(self, model: str, side: str) -> Path:
        urdf_model = _URDF_MODEL_ALIASES.get(model, model)
        path = self.assets_dir / f"{urdf_model}_{side}" / f"realhand_{urdf_model}_{side}.urdf"
        if not path.exists():
            raise FileNotFoundError(f"URDF path does not exist: {path}")
        return path

    @staticmethod
    def _normalise_model(model: str) -> str:
        key = model.lower()
        if key not in REALHAND_MODEL_LENGTHS:
            supported = ", ".join(sorted(REALHAND_MODEL_LENGTHS))
            raise ValueError(f"Unsupported RealHand model '{model}'. Supported models: {supported}")
        return key

    @staticmethod
    def _config_model(model: str) -> str:
        return "l10" if model == "l20lite" else model

    @staticmethod
    def _read_yaml(path: Path) -> dict[str, Any]:
        if yaml is None:
            raise ImportError("PyYAML is required for RealHandCore: pip install pyyaml")
        with path.open("r", encoding="utf-8") as handle:
            return yaml.safe_load(handle)

    @staticmethod
    def _change_list(values: list[Any]) -> list[Any]:
        return [None if value == "None" else value for value in values]

    @staticmethod
    def _scale_value(original_value: float, a_min: float, a_max: float, b_min: float, b_max: float) -> float:
        if a_max == a_min:
            return b_min
        return (original_value - a_min) * (b_max - b_min) / (a_max - a_min) + b_min

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return min(max_value, max(min_value, value))
