import os
import sys
import time
import threading

sys.path.append('../lib')

from unitree_actuator_sdk import *

from motor_manager.A1_motor import A1Motor
from motor_manager.motor_manager import MotorManager

# init manager with 20ms(50hz)
manager = MotorManager(20)

# A1电机请用扩展坞接到Orin左下角的USB插口，并不要交换四个转接器的位置！
# 串口ID：0左腿，1左髋，2右髋，3右腿
# motor： 1左腿侧抬腿，2左膝盖前后，3左脚踝,4左腿旋转，5左大腿上抬，6右腿旋转，7右大腿上抬，8右腿侧抬腿，9右膝盖前后，10右脚踝
motor_test = A1Motor("/dev/my485serial0", 2, kp=0.2, kd=0.01)
motor_test_name = manager.register_motor(motor_test)
target = 0.1

def process_motor_data(name_to_motor_data: dict[str, MotorData]):
    for name, data in name_to_motor_data.items():
        print(f"{name}, tau: {data.tau}, temp: {data.temp}, dq: {data.dq}, q:{data.q}")

manager.add_motor_data_callback(process_motor_data)

def send_motor_target():
    global target
    motor_test.motor_mode = MotorMode.FOC
    while True:
        if motor_test.q - target > 0.1:
            motor_test.q = motor_test.q - 0.05
        elif target - motor_test.q > 0.1:
            motor_test.q = motor_test.q + 0.05
        print(target, motor_test.q)
        time.sleep(0.02)

def change_target():
    global target
    while True:
        time.sleep(1)
        target = 1.0
        time.sleep(1)
        target = 0.1


send_motor_target_thread = threading.Thread(target=send_motor_target)
change_target_thread = threading.Thread(target=change_target)
motor_manager_thread = threading.Thread(target=manager.run)

send_motor_target_thread.start()
change_target_thread.start()
motor_manager_thread.start()

send_motor_target_thread.join()
change_target_thread.join()
motor_manager_thread.join()

manager.stop()
