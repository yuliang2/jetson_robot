import os
import sys
import time
import threading
from collections import deque
import math

import cv2

sys.path.append('../lib')

from unitree_actuator_sdk import *
from motor_manager.A1_motor import A1Motor
from motor_manager.motor_manager import MotorManager

from hpe.yolo_pose import PoseEstimator

# 电机配置列表（根据实际硬件连接修改以下配置）
MOTOR_CONFIGS = [
    # 格式：serial_port, motor_id, kp, kd, init_pos, min_limit, max_limit, motor_name
    ("/dev/my485serial0", 0, 0.2, 0.01, 0.227, 0.1, 0.7, "左腿侧抬腿"),
    ("/dev/my485serial0", 1, 0.2, 0.01, 0.429, -0.25, 1.3, "left_knee"),
    ("/dev/my485serial0", 2, 0.2, 0.01, 0.095, -0.5, 0.8, "左脚踝"),
    ("/dev/my485serial1", 0, 0.2, 0.01, 0.137, -0.2, 0.4, "左腿旋转"),
    ("/dev/my485serial1", 1, 0.2, 0.01, 0.177, 0.1, 0.8, "左大腿上抬"),
    ("/dev/my485serial2", 0, 0.2, 0.01, 0.667, 0.5, 0.7, "右腿旋转"),
    ("/dev/my485serial2", 1, 0.2, 0.01, 0.537, -0.1, 0.6, "右大腿上抬"),
    ("/dev/my485serial3", 0, 0.2, 0.01, 0.100, 0.1, 0.7, "右腿侧抬腿"),
    ("/dev/my485serial3", 1, 0.2, 0.01, 0.154, -0.7, 0.8, "right_knee"),
    ("/dev/my485serial3", 2, 0.2, 0.01, 0.108, -0.8, 0.5, "右脚踝")
]


class EnhancedMotorController:
    def __init__(self, manager):
        self.manager = manager
        self.motors = []
        self.targets = {}
        self.lock = threading.Lock()
        self.command_queue = deque(maxlen=10)

        # 初始化所有电机
        for config in MOTOR_CONFIGS:
            motor = A1Motor(
                serial_path=config[0],
                id=config[1],
                kp=config[2],
                kd=config[3],
            )
            motor_name = manager.register_motor(motor)
            self.motors.append({
                "obj": motor,
                "name": motor_name,
                "init_pos": config[4],
                "min_limit": config[5],
                "max_limit": config[6],
                "desc": config[7]
            })
            self.targets[motor_name] = config[4]

        # 设置初始位置
        self.reset_all_positions()

    def reset_all_positions(self):
        """将所有电机复位到初始位置"""
        with self.lock:
            for motor in self.motors:
                motor["obj"].q = motor["init_pos"]
                self.targets[motor["name"]] = motor["init_pos"]
                motor["obj"].motor_mode = MotorMode.FOC

    def update_targets(self, new_targets):
        """批量更新目标位置（带限位保护）"""
        with self.lock:
            for name, target in new_targets.items():
                motor = next(m for m in self.motors if m["name"] == name)
                # 应用位置限位
                self.targets[name] = max(
                    motor["min_limit"],
                    min(target, motor["max_limit"])
                )

    def get_motor_data(self):
        """获取当前所有电机数据（线程安全）"""
        with self.lock:
            return {m["name"]: m["obj"].q for m in self.motors}


# 初始化管理器（20ms周期）
manager = MotorManager(20)
controller = EnhancedMotorController(manager)


# 数据回调函数（优化输出格式）
def process_motor_data(name_to_motor_data: dict[str, MotorData]):
    os.system('cls' if os.name == 'nt' else 'clear')
    print("{:<12} | {:<8} | {:<8} | {:<8}".format("Motor", "Position", "Target", "Temp"))
    print("-" * 45)
    for name, data in name_to_motor_data.items():
        motor = next(m for m in controller.motors if m["name"] == name)
        print("{:<12} | {:>8.2f} | {:>8.2f} | {:>8.1f}℃".format(
            motor["desc"],
            data.q,
            controller.targets[name],
            data.temp
        ))


manager.add_motor_data_callback(process_motor_data)


def control_loop():
    """优化后的控制循环（100Hz）"""
    while True:
        start_time = time.time()

        # 批量生成控制指令
        commands = {}
        for motor in controller.motors:
            current_q = motor["obj"].q
            target = controller.targets[motor["name"]]

            # 生成平滑运动指令
            if abs(current_q - target) > 0.1:
                new_q = current_q + 0.05 * (1 if target > current_q else -1)
            else:
                new_q = target

            # 应用限位保护
            new_q = max(motor["min_limit"], min(new_q, motor["max_limit"]))
            commands[motor["name"]] = new_q

        # 批量提交指令
        with controller.lock:
            for motor in controller.motors:
                motor["obj"].q = commands[motor["name"]]

        # 维持固定频率
        elapsed = time.time() - start_time
        sleep_time = max(0, 0.01 - elapsed)
        time.sleep(sleep_time)


def target_generator():
    """目标生成器（支持多种运动模式）"""
    pattern = -11
    while True:
        time.sleep(3)  # 每3秒切换模式

        new_targets = {}
        if pattern == 0:  # 同步正弦波
            phase = time.time() * 2
            for motor in controller.motors:
                new_targets[motor["name"]] = motor["init_pos"] + 0.5 * math.sin(phase)
        elif pattern == 1:  # 交替运动
            for i, motor in enumerate(controller.motors):
                new_targets[motor["name"]] = motor["init_pos"] + (0.5 if i % 2 else -0.5)
        else:  # 复位模式
            for motor in controller.motors:
                new_targets[motor["name"]] = motor["init_pos"]

        controller.update_targets(new_targets)
        # pattern = (pattern + 1) % 3

def pos_target_generator():
    pose_estimator = PoseEstimator()
    camera_index = 4
    cap = cv2.VideoCapture(camera_index)
    cap.set(5, 30)  # 帧率
    count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        if count % 5 == 0:

            annotated_frame, angles = pose_estimator.inference(frame)

            cv2.imshow("YOLO-Pose", annotated_frame)
            print(angles)
            if angles:
                new_targets = {}
                for motor in controller.motors:
                    if motor["name"] in angles:
                        new_targets[motor["name"]] = (angles[motor["name"]]-180)/180.0*math.pi + motor["init_pos"]
                controller.update_targets(angles)
        count += 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()




# 创建并启动线程
threads = [
    threading.Thread(target=control_loop, daemon=True),
    # threading.Thread(target=target_generator, daemon=True),
    threading.Thread(target=manager.run, daemon=True),
    threading.Thread(target=pos_target_generator, daemon=True),
]

for t in threads:
    t.start()

# 主线程处理异常和退出
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    manager.stop()
    print("\n停止所有电机...")
