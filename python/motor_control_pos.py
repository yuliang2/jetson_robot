import os
import sys
import time
import multiprocessing as mp
from multiprocessing import Process, Queue
from collections import deque
import math
import cv2
import pyrealsense2 as rs
import numpy as np

sys.path.append('../lib')

from unitree_actuator_sdk import *
from motor_manager.A1_motor import A1Motor
from motor_manager.motor_manager import MotorManager
from hpe.yolo_pose import PoseEstimator
from com_with_mcu import *

# 共享电机配置（使用Manager创建共享字典）
MOTOR_CONFIGS = [
    # 格式：serial_port, motor_id, kp, kd, init_pos, min_limit, max_limit, motor_name
    ("/dev/my485serial0", 0, 0.2, 0.01, 0.227, 0.1, 0.7, "左腿侧抬腿"),
    ("/dev/my485serial0", 1, 0.2, 0.01, 0.429, -0.25, 1.8, "left_knee"),
    ("/dev/my485serial0", 2, 0.2, 0.01, 0.095, -0.5, 0.8, "左脚踝"),
    ("/dev/my485serial1", 0, 0.2, 0.01, 0.137, -0.2, 0.4, "左腿旋转"),
    ("/dev/my485serial1", 1, 0.2, 0.01, 0.177, 0.1, 0.8, "左大腿上抬"),
    ("/dev/my485serial2", 0, 0.2, 0.01, 0.667, 0.5, 0.7, "右腿旋转"),
    ("/dev/my485serial2", 1, 0.2, 0.01, 0.537, -0.1, 0.6, "右大腿上抬"),
    ("/dev/my485serial3", 0, 0.2, 0.01, 0.100, 0.1, 0.7, "右腿侧抬腿"),
    ("/dev/my485serial3", 1, 0.2, 0.01, 0.154, -0.7, 0.8, "right_knee"),
    ("/dev/my485serial3", 2, 0.2, 0.01, 0.108, -0.8, 0.5, "右脚踝")
]

mcu_command_q = Queue(maxsize=10)


class EnhancedMotorController:
    def __init__(self, manager, shared_targets):
        self.manager = manager
        self.motors = []
        self.shared_targets = shared_targets  # 共享目标字典
        self.lock = mp.Lock()
        self.command_queue = deque(maxlen=10)

        # 初始化电机（与原始代码相同）
        for config in MOTOR_CONFIGS:
            motor = A1Motor(
                serial_path=config[0],
                id=config[1],
                kp=config[2],
                kd=config[3],
                motor_name=config[7],
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
            self.shared_targets[motor_name] = config[4]

        self.reset_all_positions()

    def reset_all_positions(self):
        with self.lock:
            for motor in self.motors:
                motor["obj"].q = motor["init_pos"]
                self.shared_targets[motor["name"]] = motor["init_pos"]
                motor["obj"].motor_mode = MotorMode.FOC

    def update_targets(self, new_targets):
        with self.lock:
            for name, target in new_targets.items():
                motor = next(m for m in self.motors if m["name"] == name)
                self.shared_targets[name] = max(
                    motor["min_limit"],
                    min(target, motor["max_limit"])
                )


# 子进程函数定义
def control_process(shared_targets):
    manager = MotorManager(50)
    controller = EnhancedMotorController(manager, shared_targets)

    def _process_motor_data(name_to_motor_data):
        # os.system('cls' if os.name == 'nt' else 'clear')
        print("{:<12} | {:<8} | {:<8} | {:<8}".format("Motor", "Position", "Target", "Temp"))
        print("-" * 45)
        for name, data in name_to_motor_data.items():
            motor = next(m for m in controller.motors if m["name"] == name)
            print("{:<12} | {:>8.2f} | {:>8.2f} | {:>8.1f}℃".format(
                motor["desc"],
                data.q,
                shared_targets[name],
                data.temp
            ))

    manager.add_motor_data_callback(_process_motor_data)
    manager.run()

    while True:
        start_time = time.time()
        commands = {}
        for motor in controller.motors:
            current_q = motor["obj"].q
            target = shared_targets[motor["name"]]

            if abs(current_q - target) > 0.1:
                new_q = current_q + 0.05 * (1 if target > current_q else -1)
            else:
                new_q = target

            new_q = max(motor["min_limit"], min(new_q, motor["max_limit"]))
            motor["obj"].q = new_q

        elapsed = time.time() - start_time
        time.sleep(max(0.0, 0.01 - elapsed))


def video_process(shared_targets):
    pose_estimator = PoseEstimator()
    # cap = cv2.VideoCapture(4)
    # cap.set(5, 30)

    cap = cv2.VideoCapture("hpe/test_video1.mp4")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: continue

        annotated_frame, angles = pose_estimator.inference(frame)
        cv2.imshow("YOLO-Pose", annotated_frame)

        if angles:
            new_targets = {}
            for config in MOTOR_CONFIGS:
                name = config[7]
                if name in angles:
                    new_targets[name] = math.radians(angles[name] - 180) + config[4]

            # 更新共享目标（无需额外加锁）
            shared_targets.update(new_targets)
            print(shared_targets)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def realsense_process(shared_targets):
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The program requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pose_estimator = PoseEstimator()

    # Start streaming
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            annotated_frame, angles = pose_estimator.inference(color_image)
            if angles:
                print(angles)
                new_targets = {}
                for config in MOTOR_CONFIGS:
                    name = config[7]
                    if name in angles:
                        new_targets[name] = math.radians(angles[name] - 180)*2 + config[4]

                if 'left_elbow' in angles:
                    new_targets['A'] = int(-(angles['left_elbow'] - 180)/180.0*2048.0 + 2266)

                if 'right_elbow' in angles:
                    new_targets['E'] = int(-(angles['right_elbow'] - 180)/180.0*2048.0 + 2178)


                # 更新共享目标（无需额外加锁）
                shared_targets.update(new_targets)
                print(shared_targets)

            # Show images
            cv2.imshow('annotated_frame', annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop streaming
        pipeline.stop()

def mcu_process(shared_targets, command_q):
    arm_targets = {'A': 2266, 'B': 1724, 'C': 3800, 'D': 1519, 'E': 2178, 'F': 1227, 'G': 1706, 'H': 2070}
    shared_targets.update(arm_targets)
    command_q.put(arm_targets)

    lock = mp.Lock()
    try:
        while True:
            with lock:
                if 'A' in shared_targets:
                    arm_targets['A'] = shared_targets['A']
                if 'E' in shared_targets:
                    arm_targets['E'] = shared_targets['E']
            command_q.put(arm_targets)
            print(arm_targets)
            time.sleep(0.1)

    finally:
        arm_targets = {'A': 2266, 'B': 1724, 'C': 3800, 'D': 1519, 'E': 2178, 'F': 1227, 'G': 1706, 'H': 2070}
        command_q.put(arm_targets)


if __name__ == '__main__':
    # 创建共享数据
    with mp.Manager() as manager:
        shared_targets = manager.dict()
        command_q = Queue(maxsize=10)
        com_with_mcu = ComWithMCU(port="/dev/my485serial_mcu", command_q=command_q, output=True)
        # 创建进程
        processes = [
            mp.Process(target=control_process, args=(shared_targets,)),
            # mp.Process(target=video_process, args=(shared_targets,))
            mp.Process(target=realsense_process, args=(shared_targets,)),
            mp.Process(target=mcu_process, args=(shared_targets,command_q)),
        ]

        # 启动进程
        for p in processes:
            p.daemon = True
            p.start()

        com_with_mcu.start()

        # 主进程监控
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n停止所有电机...")
            # 此处需要添加电机安全停止逻辑
            for p in processes:
                p.terminate()
