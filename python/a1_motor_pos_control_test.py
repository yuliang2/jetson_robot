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
                  mode=params.ModeType.STOP, reduction_ratio=9.2, pos_init_offset=0.8)
motor3 = A1_Motor(serial_id=0, motor_id=2,
                  mode=params.ModeType.STOP, reduction_ratio=9.2, pos_init_offset=1.4)

motor4 = A1_Motor(serial_id=1, motor_id=0,
                mode=params.ModeType.STOP, reduction_ratio=9.2)
motor5 = A1_Motor(serial_id=1, motor_id=1,
                  mode=params.ModeType.STOP, reduction_ratio=9.2, pos_init_offset=3/9.2)

motor6 = A1_Motor(serial_id=2, motor_id=0,
                mode=params.ModeType.STOP, reduction_ratio=9.2)
motor7 = A1_Motor(serial_id=2, motor_id=1,
                mode=params.ModeType.STOP, reduction_ratio=9.2, pos_init_offset=-0.32)

motor8 = A1_Motor(serial_id=3, motor_id=0,
                mode=params.ModeType.STOP, reduction_ratio=9.2)
motor9 = A1_Motor(serial_id=3, motor_id=1,
                mode=params.ModeType.STOP, reduction_ratio=9.2, pos_init_offset=-0.8)
motor10 = A1_Motor(serial_id=3, motor_id=2,
                mode=params.ModeType.STOP, reduction_ratio=9.2, pos_init_offset=-1.4)

# motor： 1左腿侧抬腿，2左膝盖前后，3左脚踝,4左腿旋转，5左大腿上抬，6右腿旋转，7右大腿上抬，8右腿侧抬腿，9右膝盖前后，10右脚踝

def motor_pos_init(motor):
    motor.SetParams(0.002, 2)

    motor.ReadData()
    start_time = time.time()
    start_pos = motor.data.q / motor.reduction_ratio
    init_direction = 1 if motor.pos_init_offset<0 else -1 # 电机位置初始化的转动方向与offset方向相反
    while True:
        motor.AbsPosControlWithoutOffset(0.02, start_pos + (time.time() - start_time) * init_direction)
        print('motor{}:Pos:{:.3f}, tau:{}, LW:{}'.format(1, motor.data.q / motor.reduction_ratio, motor.data.tau, motor.data.dq))
        if abs(motor.data.tau) > 0.15 and time.time() - start_time > 0.2:
            motor.MotorStop()
            break
        if time.time() - start_time > 5:
            motor.MotorStop()
            print("motor init error")
            exit(1)
        time.sleep(0.0002)  # 200 us

    motor.pos_offset = motor.pos_init_offset + motor.data.q / motor.reduction_ratio
    print("motor_data_q", motor.data.q)
    print("motor_pos_offset:", motor.pos_offset)

    start_time = time.time()
    while time.time() - start_time < 1:
        motor.AbsPosControl(0.01, 0)
        # motor1.ReadData()
        print('motor{}:Pos:{:.3f}, tau:{}, LW:{}'.format(1, motor.data.q / motor.reduction_ratio, motor.data.tau, motor.data.dq))
        time.sleep(0.0002)  # 200 us

    # 关闭电机
    # motor.MotorStop()

if __name__ == '__main__':
    time.sleep(1)
    motor1.AbsPosControl(0.01,0.2)
    motor_pos_init(motor2)
    motor4.AbsPosControl(0.01,0.042)
    motor_pos_init(motor5)
    motor6.AbsPosControl(0.01,0.192)
    motor_pos_init(motor7)
    motor8.AbsPosControl(0.01, 0.7)
    motor_pos_init(motor9)
    motor_pos_init(motor10)
