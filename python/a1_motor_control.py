# https://support.unitree.com/home/zh/Motor_SDK_Dev_Guide/Python_Example

import time
import sys
from a1_parameters_setting import A1_Params
sys.path.append('../lib')
from unitree_actuator_sdk import *


class A1_Motor:
    r"""
    A1 Motor Class.
    Author@ZZJ

    Args:
        serial_id (0, ): ID of serials, can show by running `cd /dev` and `ls | grep ttyUSB`.
        motor_id (0, 1, 2): ID of A1 motor, can only be 0, 1, 2. Therefore, only 3 motors can \
            be mounted on a serial device.
        motor_mode (A1_Params.ModeType): 0 -- stop, 10 -- FOC control. When the value of `motor_mode` is 0, \
            the motor enters brake mode. At this point the other control parameters have no effect \
            and the motor will stop rotating. 
    """
    def __init__(self, serial_id, motor_id, mode):
        assert serial_id in [0, 1, 2, 3], "serial_id should be 0, 1, 2 or 3"
        assert motor_id in [0, 1, 2], "motor_id should be 0, 1 or 2"
        assert mode in [0, 10], "mode should be 0-stop or 10-FOC"

        self.motor_id = motor_id
        self.serial_name = f'/dev/ttyUSB{serial_id}'
        self.serial = SerialPort(self.serial_name)
        self.cmd = MotorCmd()
        self.data = MotorData()

        self.data.motorType = MotorType.A1
        self.cmd.motorType = MotorType.A1
        self.cmd.mode = mode
        self.cmd.id   = 0
        self.cmd.q    = 0.0
        self.cmd.dq   = 0.0
        self.cmd.kp   = 0.0
        self.cmd.kd   = 0
        self.cmd.tau  = 0.0
    
    def SetLimitPos(self, max_angle, min_angle):
        assert max_angle <= 1024, "max_angle should less or equal to 1024"
        assert min_angle >= 0, "max_angle should more or equal to 0"
        self.max_angle = max_angle
        self.min_angle = min_angle
    
    def SetLimitSpeed(self, max_speed, min_speed):
        assert max_speed <= 1024, "max_speed should less or equal to 1024"
        assert min_speed >= 0, "min_speed should more or equal to 0"
        self.max_speed = max_speed
        self.min_speed = min_speed

    def PosControl(self, pos):
        r"""
        Cotrol Motor by position. And can refresh the latest motor status.

        Args:
            pos (float, 0-1024): aim position.
        Return:
            True: Set success.
            False: Set failed, target position exceeded limits.
        """
        # TODO: other params, auto control?
        if pos > self.max_angle or pos < self.min_angle:
            return False
        self.cmd.mode = 10  # FOC
        self.cmd.q = pos
        self.serial.sendRecv(self.cmd, self.data)






params = A1_Params()
motor1 = A1_Motor(serial_id=0, motor_id=0, 
                  mode=params.ModeType.STOP)

motor1.PosControl(5)
