import time
import logging

from unitree_actuator_sdk import *

logger = logging.getLogger(__file__.split("/")[-1])


class MotorInstance(object):
    def get_motor_name(self):
        pass

    def get_motor_cmd(self):
        pass

    def get_motor_data(self):
        pass

    def readonly(self):
        pass

    def sendrecv(self, cmd: MotorCmd) -> MotorData:
        pass

    def reset(self):
        pass
