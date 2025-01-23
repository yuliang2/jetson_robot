# Mock sdk interface for development

import enum


class MotorType(enum.Enum):
    A1 = enum.auto()
    B1 = enum.auto()
    GO_M8010_6 = enum.auto()


class MotorMode(enum.Enum):
    BRAKE = enum.auto()
    FOC = enum.auto()
    CALIBRATE = enum.auto()


class MotorCmd(object):
    motorType: MotorType
    hex_len: int
    id: int
    mode: int
    tau: float  # T
    dq: float  # W
    q: float  # pos
    kp: float  # K_P
    kd: float  # K_W


class MotorData(object):
    motorType: MotorType
    hex_len: int
    motor_id: int
    mode: int
    temp: int
    merror: int
    tau: float
    dq: float
    q: float
    correct: bool


class SerialPort(object):
    def test(self): ...
    def sendRecv(cmd: MotorCmd, data: MotorData) -> bool: ...


def queryMotorMode(motortype: MotorType, motormode: MotorMode): ...
def queryGearRatio(motortype: MotorType): ...
