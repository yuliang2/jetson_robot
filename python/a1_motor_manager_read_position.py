import os
import sys
import time

sys.path.append('../lib')

from unitree_actuator_sdk import *

from motor_manager.A1_motor import A1Motor
from motor_manager.motor_manager import MotorManager

# init manager with 20ms(50hz)
manager = MotorManager(20)

motor1 = A1Motor("/dev/my485serial0", 0, kp=0.2, kd=0.01)
motor2 = A1Motor("/dev/my485serial0", 1, kp=0.2, kd=0.01)
motor3 = A1Motor("/dev/my485serial0", 2, kp=0.2, kd=0.01)
motor4 = A1Motor("/dev/my485serial1", 0, kp=0.2, kd=0.01)
motor5 = A1Motor("/dev/my485serial1", 1, kp=0.2, kd=0.01)
motor6 = A1Motor("/dev/my485serial2", 0, kp=0.2, kd=0.01)
motor7 = A1Motor("/dev/my485serial2", 1, kp=0.2, kd=0.01)
motor8 = A1Motor("/dev/my485serial3", 0, kp=0.2, kd=0.01)
motor9 = A1Motor("/dev/my485serial3", 1, kp=0.2, kd=0.01)
motor10 = A1Motor("/dev/my485serial3", 2, kp=0.2, kd=0.01)

motors = [motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8, motor9, motor10]
for motor in motors:
    manager.register_motor(motor)


def process_motor_data(name_to_motor: dict[str, A1Motor]):
    for name, motor in name_to_motor.items():
        print(f"{name}, tau: {motor.tau}, temp: {motor.temp}, dq: {motor.dq}, q:{motor.q}")


manager.add_motor_data_callback(process_motor_data)

manager.run()

time.sleep(1000)

manager.stop()
