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


def process_motor_data(name_to_motor_data: dict[str, MotorData]):
    # do something you want, eg:
    obs_tensor = [data for _, data in name_to_motor_data.items()]
    new_action = policy(obs_tensor).detach().numpy().squeeze()
    new_cmds_dict = {}
    i = 0
    for name, motor in name_to_motor_data.items():
        new_cmds_dict[name] = new_action[i]
        i = i + 1
    manager.update_motor_cmds(new_cmds_dict)


manager.add_motor_data_callback(process_motor_data)

manager.run()

time.sleep(1000)

manager.stop()
