"""RealHand official SDK adapter for L20/G20-compatible CAN hands."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


L20_JOINT_COUNT = 20
L6_LEGACY_JOINT_COUNT = 6
L20_NEW_SDK_JOINT_COUNT = 16
DEFAULT_L20_OPEN_POSE = [250] * L20_JOINT_COUNT


@dataclass(frozen=True)
class L20SdkStatus:
    left_connected: bool
    right_connected: bool
    left_can: str | None
    right_can: str | None
    left_model: str
    right_model: str


class RealHandL20SdkController:
    """Small wrapper around the GitHub realhand SDK for L20/G20 CAN hands.

    The teleop retarget code still emits the older 20-value L20/G20 command layout:
    base pitch 5, abduction 5, thumb yaw/reserved 5, fingertip 5, all in 0-255.
    The newer SDK exposes G20-compatible hardware through the L20 class and
    expects 16 joint angles in 0-100. This adapter converts at the boundary so
    upstream teleop code can stay unchanged.
    """

    def __init__(
        self,
        *,
        left_can: str | None = "can0",
        right_can: str | None = "can1",
        left_model: str = "l20",
        right_model: str = "l20",
        connect: bool = True,
    ) -> None:
        self.left_can = left_can
        self.right_can = right_can
        self.left_model = _normalise_sdk_model(left_model)
        self.right_model = _normalise_sdk_model(right_model)
        self.left = None
        self.right = None
        if connect:
            self.connect()

    @property
    def status(self) -> L20SdkStatus:
        return L20SdkStatus(
            left_connected=self.left is not None,
            right_connected=self.right is not None,
            left_can=self.left_can,
            right_can=self.right_can,
            left_model=self.left_model,
            right_model=self.right_model,
        )

    def connect(self) -> None:
        try:
            from realhand import L6, L20
        except ImportError as exc:
            raise ImportError(
                "RealHand GitHub SDK is required: "
                "pip install git+https://github.com/RealHand-Robotics/realbot-python-sdk.git"
            ) from exc

        if self.left_can and self.left is None:
            left_class = L6 if self.left_model == "l6" else L20
            self.left = left_class(side="left", interface_name=self.left_can)
        if self.right_can and self.right is None:
            right_class = L6 if self.right_model == "l6" else L20
            self.right = right_class(side="right", interface_name=self.right_can)

    def set_speed(
        self,
        *,
        left_speed: Sequence[int] | None = None,
        right_speed: Sequence[int] | None = None,
    ) -> None:
        if left_speed is not None and self.left is not None:
            self.left.speed.set_speeds(_speed_to_sdk(left_speed, self.left_model, "left_speed"))
        if right_speed is not None and self.right is not None:
            self.right.speed.set_speeds(_speed_to_sdk(right_speed, self.right_model, "right_speed"))

    def move(
        self,
        *,
        left_pose: Sequence[int] | None = None,
        right_pose: Sequence[int] | None = None,
    ) -> None:
        if left_pose is not None:
            if self.left is None:
                raise RuntimeError("left hand is not connected")
            self.left.angle.set_angles(_pose_to_sdk_angles(left_pose, self.left_model, "left_pose"))
        if right_pose is not None:
            if self.right is None:
                raise RuntimeError("right hand is not connected")
            self.right.angle.set_angles(_pose_to_sdk_angles(right_pose, self.right_model, "right_pose"))

    def get_state(self) -> dict[str, list[int] | None]:
        return {
            "left": self.left.get_snapshot() if self.left is not None else None,
            "right": self.right.get_snapshot() if self.right is not None else None,
        }

    def close(self) -> None:
        for api in (self.left, self.right):
            if api is None:
                continue
            close_fn = getattr(api, "close", None)
            if callable(close_fn):
                close_fn()

    def __enter__(self) -> "RealHandL20SdkController":
        if self.left is None and self.right is None:
            self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def _normalise_l20_pose(values: Sequence[int], name: str) -> list[int]:
    pose = [int(round(float(v))) for v in values]
    if len(pose) == L6_LEGACY_JOINT_COUNT:
        pose = _expand_l6_legacy_to_l20_20(pose)
    if len(pose) != L20_JOINT_COUNT:
        raise ValueError(
            f"{name} must contain {L20_JOINT_COUNT} L20/G20 values "
            f"or {L6_LEGACY_JOINT_COUNT} L6 fallback values"
        )
    return [_clamp(v) for v in pose]


def l20_legacy_20_to_sdk_16(values: Sequence[int], name: str = "pose") -> list[float]:
    pose = _normalise_l20_pose(values, name)
    base = pose[0:5]
    abd = pose[5:10]
    thumb_yaw = pose[10]
    tip = pose[15:20]
    legacy_order = [
        abd[0],
        thumb_yaw,
        base[0],
        tip[0],
        abd[1],
        base[1],
        tip[1],
        abd[2],
        base[2],
        tip[2],
        abd[3],
        base[3],
        tip[3],
        abd[4],
        base[4],
        tip[4],
    ]
    return [_scale_255_to_100(value) for value in legacy_order]


def l6_legacy_6_to_sdk_6(values: Sequence[int], name: str = "pose") -> list[float]:
    pose = [int(round(float(v))) for v in values]
    if len(pose) == L20_JOINT_COUNT:
        pose = _collapse_l20_20_to_l6_6(pose)
    if len(pose) != L6_LEGACY_JOINT_COUNT:
        raise ValueError(
            f"{name} must contain {L6_LEGACY_JOINT_COUNT} L6 values "
            f"or {L20_JOINT_COUNT} L20/G20 values"
        )
    return [_scale_255_to_100(value) for value in pose]


def _pose_to_sdk_angles(values: Sequence[int], model: str, name: str) -> list[float]:
    if model == "l6":
        return l6_legacy_6_to_sdk_6(values, name)
    return l20_legacy_20_to_sdk_16(values, name)


def _expand_l6_legacy_to_l20_20(values: Sequence[int]) -> list[int]:
    """Expand L6 retarget output into the L20/G20 legacy 20-command layout.

    L6 order is treated as thumb flex, thumb spread/yaw, index, middle, ring,
    pinky. The four long fingers drive both root and tip in the 20-command
    layout so G20/L20 hardware can be tested in a reduced-DOF mode.
    """

    thumb_flex, thumb_abd, index, middle, ring, pinky = [_clamp(v) for v in values]
    pose = [255] * L20_JOINT_COUNT
    pose[0:5] = [thumb_flex, index, middle, ring, pinky]
    pose[5:10] = [thumb_abd, 127, 127, 127, 127]
    pose[10] = thumb_abd
    pose[15:20] = [thumb_flex, index, middle, ring, pinky]
    return pose


def _collapse_l20_20_to_l6_6(values: Sequence[int]) -> list[int]:
    pose = [_clamp(int(round(float(v)))) for v in values]
    return [pose[0], pose[5], pose[1], pose[2], pose[3], pose[4]]


def _expand_l20_speed(values: Sequence[int], name: str) -> list[float]:
    data = [float(v) for v in values]
    if len(data) != 5:
        raise ValueError(f"{name} must contain exactly 5 values for L20")
    data = [_clamp_100(v) for v in data]
    thumb, index, middle, ring, pinky = data
    return [
        thumb,
        thumb,
        thumb,
        thumb,
        index,
        index,
        index,
        middle,
        middle,
        middle,
        ring,
        ring,
        ring,
        pinky,
        pinky,
        pinky,
    ]


def _expand_l6_speed(values: Sequence[int], name: str) -> list[float]:
    data = [float(v) for v in values]
    if len(data) == 5:
        thumb, index, middle, ring, pinky = data
        data = [thumb, thumb, index, middle, ring, pinky]
    if len(data) != 6:
        raise ValueError(f"{name} must contain 5 grouped values or 6 L6 values")
    return [_clamp_100(v) for v in data]


def _speed_to_sdk(values: Sequence[int], model: str, name: str) -> list[float]:
    if model == "l6":
        return _expand_l6_speed(values, name)
    return _expand_l20_speed(values, name)


def _normalise_sdk_model(model: str) -> str:
    key = model.lower()
    if key == "auto":
        key = "l20"
    if key == "g20":
        key = "l20"
    if key not in {"l6", "l20"}:
        raise ValueError(f"unsupported SDK hand model {model!r}; use l6, l20, or g20")
    return key


def _clamp(value: int) -> int:
    return max(0, min(255, int(value)))


def _clamp_100(value: float) -> float:
    return max(0.0, min(100.0, float(value)))


def _scale_255_to_100(value: int) -> float:
    return round(_clamp(value) * 100.0 / 255.0, 3)
