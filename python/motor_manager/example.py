import os
import time

from unitree_actuator_sdk import *

from A1_motor import A1Motor
from motor_manager import MotorManager

# init manager with 20ms(50hz)
manager = MotorManager(20)

motor00_name = manager.register_motor(A1Motor("/dev/my485serial0", 0))
motor01_name = manager.register_motor(A1Motor("/dev/my485serial0", 1))

motor10_name = manager.register_motor(A1Motor("/dev/my485serial1", 0))


def process_motor_response(name_to_motor: dict[str, A1Motor]):
    # do something you want, eg:
    obs_tensor = [motor.q for _, motor in name_to_motor.items()]
    new_action = policy(obs_tensor).detach().numpy().squeeze()
    i = 0
    for motor in name_to_motor.values():
        motor.q = new_action[i]
        i += 1


manager.add_motor_data_callback(process_motor_response)

manager.run()

time.sleep(1000)

manager.stop()
