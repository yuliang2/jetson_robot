import os
import sys
import time

sys.path.append('../lib')

from unitree_actuator_sdk import *

from motor_manager.A1_motor import A1Motor
from motor_manager.motor_manager import MotorManager

# init manager with 20ms(50hz)
manager = MotorManager(20)

# A1电机请用扩展坞接到Orin左下角的USB插口，并不要交换四个转接器的位置！
# 串口ID：0左腿，1左髋，2右髋，3右腿
# motor： 1左腿侧抬腿，2左膝盖前后，3左脚踝,4左腿旋转，5左大腿上抬，6右腿旋转，7右大腿上抬，8右腿侧抬腿，9右膝盖前后，10右脚踝
motor10 = A1Motor("/dev/my485serial0", 3, kp=0.2, kd=0.01, mode=MotorMode.FOC)

motor10_name = manager.register_motor(motor10)
# motor01_name = manager.register_motor(A1Motor("/dev/my485serial0", 1))
#
# motor10_name = manager.register_motor(A1Motor("/dev/my485serial1", 0))

# test_cmd = motor10.motor_cmd
# test_cmd.q = 1
# manager.update_motor_cmds({motor10_name: test_cmd})
# exit(0)

def process_motor_data(name_to_motor_data: dict[str, MotorData]):
    for name, data in name_to_motor_data.items():
        print(f"{name}, tau: {data.tau}, temp: {data.temp}, dq: {data.dq}, q:{data.q}")
    # do something you want, eg:
    # obs_tensor = [data for _, data in name_to_motor_data.items()]
    # new_action = policy(obs_tensor).detach().numpy().squeeze()
    # new_cmds_dict = {}
    # i = 0
    # for name, motor in name_to_motor_data.items():
    #     new_cmds_dict[name] = new_action[i]
    #     i = i + 1
    # manager.update_motor_cmds(new_cmds_dict)


manager.add_motor_data_callback(process_motor_data)

manager.run()

time.sleep(1000)

manager.stop()
