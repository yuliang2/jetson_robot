import time
import logging

from unitree_actuator_sdk import *

from motor_manager.utils import timeit

from motor_manager.motor_instance import MotorInstance

logger = logging.getLogger(__file__.split("/")[-1])


class A1Motor(MotorInstance):
    motor_name: str
    serial_path: str
    id: int
    _motor_mode: MotorMode
    motor_type: MotorType
    serial: SerialPort

    motor_cmd: MotorCmd = MotorCmd()
    motor_data: MotorData = MotorData()

    # motor init value
    _tau: float = 0
    _dq: float = 0
    _q: float = 0
    _kp: float = 0
    _kd: float = 0

    def __init__(
        self,
        serial_path: str,
        id: int,
        motor_name: str = None,
        motor_type: MotorType = MotorType.A1,
        tau: float = 0,
        dq: float = 0,
        q: float = 0,
        kp: float = 0,
        kd: float = 0,
    ):
        self.serial_path = serial_path
        self.motor_id = id
        self._motor_mode = MotorMode.BRAKE
        self.motor_type = motor_type
        self.motor_name = "-".join([self.serial_path, str(id), str(int(time.time()))]) if motor_name is None else motor_name
        self.serial = SerialPort(self.serial_path)
        self.reduction_ratio = queryGearRatio(self.motor_type)
        self._tau = tau
        self._dq = dq
        self._q = q
        self._kp = kp
        self._kd = kd
        self.init_motor_cmd()
        self.init_motor_data()

    def init_motor_cmd(self):
        motor_cmd = MotorCmd()
        motor_cmd.motorType = self.motor_type
        motor_cmd.mode = queryMotorMode(self.motor_type, self._motor_mode)
        motor_cmd.id = self.motor_id
        self.motor_cmd = motor_cmd
        self.tau = self._tau
        self.dq = self._dq
        self.q = self._q
        self.kp = self._kp
        self.kd = self._kd

    def init_motor_data(self):
        motor_data = MotorData()
        motor_data.motorType = self.motor_type
        motor_data.mode = queryMotorMode(self.motor_type, self._motor_mode)
        motor_data.motor_id = self.motor_id
        motor_data.tau = 0
        motor_data.dq = 0
        motor_data.q = 0
        motor_data.temp = 0
        self.motor_data = motor_data

    @property
    def motor_mode(self):
        return self.motor_data.mode

    @motor_mode.setter
    def motor_mode(self, value):
        value = queryMotorMode(self.motor_type, value)
        self.motor_cmd.mode = value

    @property
    def tau(self):
        return self.motor_data.tau

    @tau.setter
    def tau(self, value):
        if abs(value) < 128:
            self.motor_cmd.tau = value
        else:
            raise Exception(f"tau value invalid: {value}")

    @property
    def dq(self):
        return self.motor_data.dq

    @dq.setter
    def dq(self, value):
        if abs(value * self.reduction_ratio) < 256:
            self.motor_cmd.dq = value * self.reduction_ratio
        else:
            raise Exception(f"dq value invalid: {value}")

    @property
    def q(self):
        return self.motor_data.q / self.reduction_ratio

    @q.setter
    def q(self, value):
        if abs(value * self.reduction_ratio) < 823549:
            self.motor_cmd.q = value * self.reduction_ratio
        else:
            raise Exception(f"q value invalid: {value}")

    KP_MAGIC_NUM = 26.07
    @property
    def kp(self):
        return self.motor_cmd.kp * A1Motor.KP_MAGIC_NUM

    @kp.setter
    def kp(self, value):
        value = value / A1Motor.KP_MAGIC_NUM
        if 0 <= value and value < 16:
            self.motor_cmd.kp = value
        else:
            raise Exception(f"kp value invalid: {value}")

    KD_MAGIC_NUM = 100
    @property
    def kd(self):
        return self.motor_cmd.kd / A1Motor.KD_MAGIC_NUM

    @kd.setter
    def kd(self, value):
        value = value * A1Motor.KD_MAGIC_NUM
        if 0 <= value and value < 32:
            self.motor_cmd.kd = value
        else:
            raise Exception(f"kd value invalid: {value}")

    @property
    def temp(self):
        return self.motor_data.temp

    def get_motor_cmd(self):
        return self.motor_cmd

    def get_motor_data(self):
        return self.motor_data

    @timeit
    def reset(self):
        self.init_motor_cmd()
        self.init_motor_data()
        self.readonly()

    @timeit
    def readonly(self):
        self.sendrecv(self.motor_cmd)

    def get_motor_name(self):
        return self.motor_name

    def set_pos(self, pos):
        self.q = pos
        self.sendrecv(self.motor_cmd)

    @timeit
    def sendrecv(self, cmd: MotorCmd) -> MotorData:
        self.serial.sendRecv(cmd, self.motor_data)
        return self.motor_data
