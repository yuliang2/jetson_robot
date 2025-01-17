import time
from a1_motor_control import *

params = A1_Params()
motor1 = A1_Motor(serial_id=0, motor_id=0, 
                mode=params.ModeType.STOP, reduction_ratio=9.2)
motor2 = A1_Motor(serial_id=0, motor_id=1, 
                mode=params.ModeType.STOP, reduction_ratio=9.2)
motor3 = A1_Motor(serial_id=0, motor_id=2, 
                mode=params.ModeType.STOP, reduction_ratio=9.2)

motor4 = A1_Motor(serial_id=1, motor_id=0, 
                mode=params.ModeType.STOP, reduction_ratio=9.2)
motor5 = A1_Motor(serial_id=1, motor_id=1, 
                mode=params.ModeType.STOP, reduction_ratio=9.2)

motor6 = A1_Motor(serial_id=2, motor_id=0, 
                mode=params.ModeType.STOP, reduction_ratio=9.2)
motor7 = A1_Motor(serial_id=2, motor_id=1, 
                mode=params.ModeType.STOP, reduction_ratio=9.2)

motor8 = A1_Motor(serial_id=3, motor_id=0, 
                mode=params.ModeType.STOP, reduction_ratio=9.2)
motor9 = A1_Motor(serial_id=3, motor_id=1, 
                mode=params.ModeType.STOP, reduction_ratio=9.2)
motor10 = A1_Motor(serial_id=3, motor_id=2, 
                mode=params.ModeType.STOP, reduction_ratio=9.2)


motors = [motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8, motor9, motor10]

while True:
    for i, motor in enumerate(motors, start=1):
        motor.ReadData()
        print('motor{}:{:.3f}'.format(i, motor.data.q / motor.reduction_ratio), end=' ')
        time.sleep(0.1)
    print('')
