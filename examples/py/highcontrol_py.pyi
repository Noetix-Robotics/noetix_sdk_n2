from __future__ import annotations
from enum import Enum
import numpy as np
from typing import List, Callable

# =====================
# Enums
# =====================

class ControlCmd(Enum):
    WALK: int
    SWING: int
    SHAKE: int
    CHEER: int
    RUN: int
    START: int
    SWITCH: int
    STARTTEACH: int
    SAVETEACH: int
    ENDTEACH: int
    PLAYTEACH: int
    DANCE1: int  # YG L
    DANCE2: int  # YG S
    DANCE3: int  # QIAQIA
    DANCE4: int  # TICAO
    DANCE5: int  # 216
    CF: int
    TANG: int
    DEFAULT: int

# =====================
# Data Classes
# =====================

class JoyData:
    def __init__(self) -> None: ...
    @property
    def axes(self) -> np.ndarray: ...
    # shape: (3,), dtype=float64
    @property
    def button(self) -> np.ndarray: ...
    # shape: (14,), dtype=int32

class NingImuData:
    def __init__(self) -> None: ...
    @property
    def ori(self) -> np.ndarray: ...
    # shape: (4,), quaternion
    @property
    def ori_cov(self) -> np.ndarray: ...
    # shape: (9,)
    @property
    def angular_vel(self) -> np.ndarray: ...
    # shape: (3,)
    @property
    def angular_vel_cov(self) -> np.ndarray: ...
    # shape: (9,)
    @property
    def linear_acc(self) -> np.ndarray: ...
    # shape: (3,)
    @property
    def linear_acc_cov(self) -> np.ndarray: ...
    # shape: (9,)

class MotorState:
    def __init__(self) -> None: ...

    pos: float
    vel: float
    tau: float
    motor_id: int
    error: int
    temperature: float

class RobotHardwareStatus:
    def __init__(self) -> None: ...
    imu_data: NingImuData
    remote_data: JoyData
    motor_data: List[MotorState]
    workmode: int

RobotHardwareStatusCallback = Callable[[RobotHardwareStatus], None]

# =====================
# Core Controller
# =====================

class HighController:
    @staticmethod
    def instance() -> HighController: ...
    def init(self) -> None: ...
    def publish_cmd(
        self, ver: float, hor: float, yaw: float, action: ControlCmd, index: int = 0
    ) -> None: ...
    def subscribe_robot_hardware_status(
        self, callback: RobotHardwareStatusCallback
    ) -> None: ...

class AoLionDriver:
    def __init__(self) -> None: ...
    def init(self, port: str, baudrate: int) -> bool: ...
    def getremotedata(self) -> JoyData: ...
