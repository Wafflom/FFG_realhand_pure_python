"""Pure Python RealHand teleop helpers.

This package intentionally has no ROS imports. It exposes the RealForce USB
serial reader/retargeter and the RealMCG UDP reader/mapper so another
controller can consume plain Python lists.
"""

from .realforce import HandSide, RealForceSerialReader, RealForcePairReader
from .realforce_retarget import RealForceCommand, RealForceRetarget
from .realhand_core import REALHAND_MODEL_LENGTHS, RealHandCore
from .l20_sdk_controller import DEFAULT_L20_OPEN_POSE, RealHandL20SdkController
from .realmcg import (
    RealHandCommand,
    RealMCGFrame,
    RealMCGMapper,
    RealMCGRetarget,
    RealMCGSerialClient,
    RealMCGUdpClient,
)

__all__ = [
    "DEFAULT_L20_OPEN_POSE",
    "HandSide",
    "REALHAND_MODEL_LENGTHS",
    "RealForceCommand",
    "RealForceSerialReader",
    "RealForcePairReader",
    "RealForceRetarget",
    "RealHandCore",
    "RealHandL20SdkController",
    "RealHandCommand",
    "RealMCGFrame",
    "RealMCGMapper",
    "RealMCGRetarget",
    "RealMCGSerialClient",
    "RealMCGUdpClient",
]
