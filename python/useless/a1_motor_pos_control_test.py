from python.useless.a1_motor_control import *

# A1电机请用扩展坞接到Orin左下角的USB插口，并不要交换四个转接器的位置！
# 串口ID：0左腿，1左髋，2右髋，3右腿
# motor： 1左腿侧抬腿，2左膝盖前后，3左脚踝,4左腿旋转，5左大腿上抬，6右腿旋转，7右大腿上抬，8右腿侧抬腿，9右膝盖前后，10右脚踝
params = A1_Params()
motor1 = A1_Motor(serial_id=0, motor_id=0,
                mode=params.ModeType.STOP, reduction_ratio=9.1)
motor2 = A1_Motor(serial_id=0, motor_id=1,
                  mode=params.ModeType.STOP, reduction_ratio=9.1, pos_init_offset=0.8)
motor3 = A1_Motor(serial_id=0, motor_id=2,
                  mode=params.ModeType.STOP, reduction_ratio=9.1, pos_init_offset=1.4)

motor4 = A1_Motor(serial_id=1, motor_id=0,
                mode=params.ModeType.STOP, reduction_ratio=9.1)
motor5 = A1_Motor(serial_id=1, motor_id=1,
                  mode=params.ModeType.STOP, reduction_ratio=9.1, pos_init_offset=3/9.1)

motor6 = A1_Motor(serial_id=2, motor_id=0,
                mode=params.ModeType.STOP, reduction_ratio=9.1)
motor7 = A1_Motor(serial_id=2, motor_id=1,
                mode=params.ModeType.STOP, reduction_ratio=9.1, pos_init_offset=-0.32)

motor8 = A1_Motor(serial_id=3, motor_id=0,
                mode=params.ModeType.STOP, reduction_ratio=9.1)
motor9 = A1_Motor(serial_id=3, motor_id=1,
                mode=params.ModeType.STOP, reduction_ratio=9.1, pos_init_offset=-0.8)
motor10 = A1_Motor(serial_id=3, motor_id=2,
                mode=params.ModeType.STOP, reduction_ratio=9.1, pos_init_offset=-1.4)

# test debug
motor11 = A1_Motor(serial_id=5, motor_id=0,
                   mode=params.ModeType.STOP, reduction_ratio=9.1, pos_init_offset=-1.4)


# motor： 1左腿侧抬腿，2左膝盖前后，3左脚踝,4左腿旋转，5左大腿上抬，6右腿旋转，7右大腿上抬，8右腿侧抬腿，9右膝盖前后，10右脚踝

def motor_pos_init(motor):
    motor.SetParams(0.2, 0.01)

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
    # time.sleep(3)
    # motor1.AbsPosControl(0.01,0.2)
    # motor_pos_init(motor2)
    # motor4.AbsPosControl(0.01,0.042)
    # motor_pos_init(motor5)
    # motor6.AbsPosControl(0.01,0.192)
    # motor_pos_init(motor7)
    # motor8.AbsPosControl(0.01, 0.7)
    # motor_pos_init(motor9)
    # motor_pos_init(motor10)

    motors = [motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8, motor9, motor10, motor11]
    for motor in motors:
        motor.ReadData()
        time.sleep(0.01)
    time.sleep(0.1)

    # motor： 1左腿侧抬腿，2左膝盖前后，3左脚踝,4左腿旋转，5左大腿上抬，6右腿旋转，7右大腿上抬，8右腿侧抬腿，9右膝盖前后，10右脚踝
    motor_test = motor11

    motor_test.SetParams(0.2/26.07, 0.01*100)
    motor_test.ReadData()
    print('motor{}:Pos:{:.3f}, tau:{}, LW:{}'.format(1, motor_test.data.q / motor_test.reduction_ratio,
                                                     motor_test.data.tau,
                                                     motor_test.data.dq))
    start_time = time.time()
    target_pos = motor_test.data.q / motor_test.reduction_ratio
    while True:
        # target_pos = start_pos + time.time() - start_time

        str_input = input("IncPos(q to exit):").strip()
        if str_input == "q":
            break
        else:
            if str_input != '':
                target_pos += eval(str_input)

            motor_test.AbsPosControlWithoutOffset(0.045, target_pos)
            # motor_test.AbsPosControlWithoutOffset(0.0, target_pos)
            print('motor: Target:{:.3f}, Pos:{:.3f}, tau:{}, LW:{}'.format(target_pos, motor_test.data.q / motor_test.reduction_ratio, motor_test.data.tau,
                                                             motor_test.data.dq))
            time.sleep(0.0002)

    # motor_test.TorqueControl(0.045)
    #
    motor_test.MotorStop()
