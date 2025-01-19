import time
from a1_motor_control import *
import math

# A1电机请用扩展坞接到Orin左下角的USB插口，并不要交换四个转接器的位置！
# 串口ID：0左腿，1左髋，2右髋，3右腿
# motor： 1左腿侧抬腿，2左膝盖前后，3左脚踝,4左腿旋转，5左大腿上抬，6右腿旋转，7右大腿上抬，8右腿侧抬腿，9右膝盖前后，10右脚踝

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
        # print('motor{}:{:.3f}'.format(i, motor.data.q), end=' ')
        time.sleep(0.1)
    print('')
