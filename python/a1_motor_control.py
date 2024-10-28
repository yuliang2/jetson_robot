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
        reduction_ratio (float): reduction ratio of motor, for A1 always be 9.1.
    """
    def __init__(self, serial_id, motor_id, mode, reduction_ratio=9.1, 
                 max_angle=None, min_angle=None, max_speed=None, min_speed=None, 
                 max_tau=None, min_tau=None):
        assert serial_id in [0, 1, 2, 3], "serial_id should be 0, 1, 2 or 3"
        assert motor_id in [0, 1, 2], "motor_id should be 0, 1 or 2"
        assert mode in [0, 10], "mode should be 0-stop or 10-FOC"

        self.motor_id = motor_id
        self.serial_name = f'/dev/ttyUSB{serial_id}'
        self.serial = SerialPort(self.serial_name)
        self.reduction_ratio = reduction_ratio

        if max_angle == None:
            self.max_angle = 823549 / self.reduction_ratio
        else:
            assert max_angle <= 823549 / self.reduction_ratio, "max_angle should less or equal to 823549 / reduction_ratio"
            assert max_angle >= - 823549 / self.reduction_ratio, "max_angle should more or equal to - 823549 / reduction_ratio"
            self.max_angle = max_angle

        if min_angle == None:
            self.min_angle = - 823549 / self.reduction_ratio
        else:
            assert min_angle <= 823549 / self.reduction_ratio, "min_angle should less or equal to 823549 / reduction_ratio"
            assert min_angle >= - 823549 / self.reduction_ratio, "min_angle should more or equal to - 823549 / reduction_ratio"
            self.min_angle = min_angle

        if max_speed == None:
            self.max_speed = 256 / self.reduction_ratio
        else:
            assert max_speed <= 256 / self.reduction_ratio, "max_speed should less or equal to 256 / reduction_ratio"
            assert max_speed >= - 256 / self.reduction_ratio, "max_speed should more or equal to - 256 / reduction_ratio"
            self.max_speed = max_speed

        if min_speed == None:
            self.min_speed = - 256 / self.reduction_ratio
        else:
            assert min_speed <= 256 / self.reduction_ratio, "min_speed should less or equal to 256 / reduction_ratio"
            assert min_speed >= - 256 / self.reduction_ratio, "min_speed should more or equal to - 256 / reduction_ratio"
            self.min_speed = min_speed

        if max_tau == None:
            self.max_tau = 128
        else:
            assert max_tau <= 128, "max_tau should less or equal to 128"
            assert max_tau >= - 128, "max_tau should more or equal to - 128"
            self.max_tau = max_tau

        if min_tau == None:
            self.min_tau = -128
        else:
            assert min_tau <= 128, "min_tau should less or equal to 128"
            assert min_tau >= - 128, "min_tau should more or equal to - 128"
            self.min_tau = min_tau

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
    
    def SetParams(self, kp, kd):
        r"""
        Refresh kp and kd.

        Args:
            kp (float, 0-16): proportional gain.
            kd (float, 0-32): derivative gain.
        Return:
            True: Set success.
            False: Set failed, kp or kd exceeded limits.
        """
        if (kp >= 0 and kp <= 16) and (kd >= 0 and kd <= 32):
            self.cmd.kp = kp
            self.cmd.kd = kd
            return True
        else:
            return False
    
    def ReadData(self):
        self.serial.sendRecv(self.cmd, self.data)
        return self.data

    def SetLimitTau(self, max_tau, min_tau):
        assert max_tau <= 128, "max_tau should less or equal to 128"
        assert max_tau >= - 128, "max_tau should more or equal to - 128"
        assert min_tau <= 128, "min_tau should less or equal to 128"
        assert min_tau >= - 128, "min_tau should more or equal to - 128"

        self.max_tau = max_tau
        self.min_tau = min_tau
    
    def SetLimitPos(self, max_angle, min_angle):
        assert max_angle <= 823549 / self.reduction_ratio, "max_angle should less or equal to 823549 / reduction_ratio"
        assert max_angle >= - 823549 / self.reduction_ratio, "max_angle should more or equal to - 823549 / reduction_ratio"
        assert min_angle <= 823549 / self.reduction_ratio, "min_angle should less or equal to 823549 / reduction_ratio"
        assert min_angle >= - 823549 / self.reduction_ratio, "min_angle should more or equal to - 823549 / reduction_ratio"

        self.max_angle = max_angle
        self.min_angle = min_angle
    
    def SetLimitSpeed(self, max_speed, min_speed):
        assert max_speed <= 256 / self.reduction_ratio, "max_speed should less or equal to 256 / reduction_ratio"
        assert max_speed >= - 256 / self.reduction_ratio, "max_speed should more or equal to - 256 / reduction_ratio"
        assert min_speed <= 256 / self.reduction_ratio, "min_speed should less or equal to 256 / reduction_ratio"
        assert min_speed >= - 256 / self.reduction_ratio, "min_speed should more or equal to - 256 / reduction_ratio"

        self.max_speed = max_speed
        self.min_speed = min_speed

    def AbsPosControl(self, tau, abs_pos):
        r"""
        Cotrol Motor by absolute position. And can refresh the latest motor status.

        Args:
            tau (float, -128 ~ 128): torque
            abs_pos (float, - 823549 / self.reduction_ratio ~ 823549 / self.reduction_ratio): aim position.
        Return:
            True: Set success.
            False: Set failed, target position exceeded limits.
        """
        if abs_pos > self.max_angle or abs_pos < self.min_angle or tau > self.max_tau or tau < self.min_tau:
            return False
        self.cmd.mode = 10  # FOC
        self.cmd.q    = abs_pos * self.reduction_ratio
        self.cmd.dq   = 0.0
        self.cmd.tau  = tau
        self.serial.sendRecv(self.cmd, self.data)
        return True

    def IncPosControl(self, tau, inc_pos):
        r"""
        Cotrol Motor by incremental position. And can refresh the latest motor status.

        Args:
            tau (float, -128 ~ 128): torque
            inc_pos (float): incremental position, incremental position + absolute position \
                should >= (- 823549 / self.reduction_ratio) and <= (823549 / self.reduction_ratio).
        Return:
            True: Set success.
            False: Set failed, target position exceeded limits.
        """
        abs_pos = self.data.q / self.reduction_ratio
        aim_abs_pos = abs_pos + inc_pos
        if aim_abs_pos > self.max_angle or aim_abs_pos < self.min_angle or tau > self.max_tau or tau < self.min_tau:
            return False
        self.cmd.mode = 10  # FOC
        self.cmd.q    = aim_abs_pos * self.reduction_ratio
        self.cmd.dq   = 0.0
        self.cmd.tau  = tau
        self.serial.sendRecv(self.cmd, self.data)
        return True


# params = A1_Params()
# motor1 = A1_Motor(serial_id=0, motor_id=0, 
#                   mode=params.ModeType.STOP, reduction_ratio=9.2)

# time.sleep(0.5)

# motor1.SetParams(0.2, 1.0)

# motor1.AbsPosControl(0, 0)
# time.sleep(1)
# motor1.ReadData()
# print('position: ', motor1.data.q/motor1.reduction_ratio)

# motor1.AbsPosControl(0, 1.57)
# time.sleep(1)
# motor1.ReadData()
# print('position: ', motor1.data.q/motor1.reduction_ratio)

# motor1.AbsPosControl(0, 3.14)
# time.sleep(1)
# motor1.ReadData()
# print('position: ', motor1.data.q/motor1.reduction_ratio)



params = A1_Params()
motor1 = A1_Motor(serial_id=0, motor_id=0, 
                  mode=params.ModeType.STOP, reduction_ratio=9.2)

time.sleep(0.5)

motor1.SetParams(0.2, 1.0)

motor1.AbsPosControl(0, 0)
time.sleep(1)
motor1.ReadData()
print('position: ', motor1.data.q/motor1.reduction_ratio)

motor1.IncPosControl(0, 0.2)
time.sleep(1)
motor1.ReadData()
print('position: ', motor1.data.q/motor1.reduction_ratio)

motor1.IncPosControl(0, -0.1)
time.sleep(1)
motor1.ReadData()
print('position: ', motor1.data.q/motor1.reduction_ratio)