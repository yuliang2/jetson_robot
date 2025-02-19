import logging.config
import time
import threading
import traceback
import os
import csv
from datetime import datetime
import logging
import yaml
import typing
from concurrent.futures import ThreadPoolExecutor, as_completed
from multiprocessing import Pool, Manager  # 新增：多进程所需
import multiprocessing

from unitree_actuator_sdk import *

from motor_manager.motor_instance import *

base_dir = os.path.dirname(__file__)
log_config = None
if not os.path.exists(base_dir + "/logs"):
    os.mkdir(base_dir + "/logs")
with open(base_dir + "/logging_config.yaml", "r") as f:
    log_config = yaml.safe_load(f.read())
    logging.config.dictConfig(log_config)

logger = logging.getLogger(__file__.split("/")[-1])

for handle in logger.handlers:
    if isinstance(handle, logging.handlers.TimedRotatingFileHandler):
        handle.suffix = "%Y-%m-%d.log"


class MotorManager(object):
    transfer_thread: threading.Thread | None = None
    cmd_interval_ms: int

    # 需要注意，下面这些 dict 默认在多进程下不会自动共享
    motor_dict: dict[str, MotorInstance] = dict()
    motor_cmds: dict[str, MotorCmd] = dict()

    loop_flag: bool = False
    motor_data: dict[str, MotorData] = dict()

    task_list: dict[str, typing.Callable] = dict()
    motor_data_callback_list: list[typing.Callable] = list()

    motor_groups: dict[str, list[str]] = dict()

    def __init__(self, cmd_interval_ms: int, max_workers: int = 4):
        self.cmd_interval_ms = cmd_interval_ms

        # ---- 使用multiprocessing.Pool代替ThreadPoolExecutor ----
        # 注意: 如果 motor_dict 很大, 会在每次apply_async时pickle/unpickle
        #       可能效率不如线程池. 请视具体情况斟酌
        self.pool = Pool(processes=max_workers)

        # multiprocessing 下的共享数据技巧:
        # 若想在子进程/主进程间共享 motor_data，可使用 Manager.dict() 等
        self.manager = Manager()           
        self.shared_motor_data = self.manager.dict()  # 用于子进程写，主进程读
        
        # 注册任务
        self.register_task(MotorManager.transfer_motor_cmds_task, task_name="TransferCmds")
        self.register_task(MotorManager.notify_motor_data_task, task_name="NotifyData")

    def __del__(self):
        try:
            self.pool.close()
            self.pool.join()
        except:
            pass

    def register_motor(self, motor: MotorInstance, group_name: str = None):
        name = motor.get_motor_name()
        self.motor_dict[name] = motor
        motor.reset()
        self.motor_cmds[name] = motor.get_motor_cmd()
        self.motor_data[name] = motor.get_motor_data()

        if group_name is None:
            group_name = "default"
        if group_name not in self.motor_groups:
            self.motor_groups[group_name] = []
        self.motor_groups[group_name].append(name)
        return name

    def get_motor(self, motor_name: str) -> MotorInstance:
        return self.motor_dict.get(motor_name)

    def register_task(self, task: typing.Callable, task_name: str = None) -> str:
        if not task_name:
            task_name = task.__name__ + "-" + str(time.time() * 1000)
        self.task_list[task_name] = task
        return task_name

    def update_motor_cmds(self, motor_cmds: dict[str, MotorCmd]):
        self.motor_cmds = motor_cmds

    @staticmethod
    def _read_group(
        motor_dict: dict[str, MotorInstance],
        group_motor_names: list[str],
        motor_cmds: dict[str, MotorCmd],
    ) -> dict[str, MotorData]:
        """
        这里改为不使用self，而是直接传入需要的数据。
        这样可以减少因为pickle self而带来的副作用。
        """
        group_data = {}
        for motor_name in group_motor_names:
            motor_instance = motor_dict.get(motor_name)
            if motor_instance is None:
                continue
            cmd = motor_cmds.get(motor_name)
            if cmd is None:
                continue
            try:
                data = motor_instance.sendrecv(cmd)
                group_data[motor_name] = data
            except Exception as e:
                logger.warning(f"Failed to read motor data for {motor_name}: {traceback.format_exc()}")
        return group_data

    @staticmethod
    def transfer_motor_cmds_task(self: "MotorManager"):
        if not self.motor_groups:
            logger.warning("No motor groups defined, skip transfer")
            return

        motor_cmds = self.motor_cmds
        # motor_data_dict 用普通 dict，最终写到 self.motor_data
        motor_data_dict = {}

        result_handles = []
        # 用多进程池并行
        for group_name, group_motor_names in self.motor_groups.items():
            # 注意: 这里传入 motor_dict, motor_cmds 时，会被pickle到子进程
            # 如果 motor_instance 无法被pickle，会出错
            # 建议只传必要信息
            async_result = self.pool.apply_async(
                MotorManager._read_group,
                (self.motor_dict, group_motor_names, motor_cmds)
            )
            result_handles.append((group_name, async_result))

        # 收集结果
        for group_name, handle in result_handles:
            try:
                group_data = handle.get()  # 阻塞等待子进程结果
                motor_data_dict.update(group_data)
            except Exception as e:
                logger.warning(f"Group {group_name} read error: {traceback.format_exc()}")

        # 更新主进程的 motor_data
        self.motor_data = motor_data_dict

        # 如果需要让别的进程访问 motor_data，可以写到 self.shared_motor_data
        # self.shared_motor_data.clear()
        # self.shared_motor_data.update(motor_data_dict)

    @staticmethod
    def notify_motor_data_task(self: "MotorManager"):
        notify_data = self.motor_data
        if not notify_data:
            return

        # 多进程并行回调：若 callback 很耗时，可以 apply_async
        # 但要确保 callback 可pickle
        # 如果 callback 只是简单的操作，也许在主进程直接调用就行
        result_handles = []
        for callback in self.motor_data_callback_list:
            # apply_async 或者直接 call
            async_res = self.pool.apply_async(callback, (notify_data,))
            result_handles.append(async_res)

        for handle in result_handles:
            try:
                handle.get()
            except Exception as e:
                logger.warning(f"notify data callback error: {traceback.format_exc()}")

    def add_motor_data_callback(self, callback: typing.Callable):
        self.motor_data_callback_list.append(callback)

    def init_record_csv(self):
        if not os.path.exists(base_dir + "/record"):
            os.mkdir(base_dir + "/record")
        datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        with open("/".join([base_dir, "record", f"{datetime_str}_record.csv"]), "w", newline="") as csvfile:
            fieldnames = ["timestamp", "motor_name", "tau", "dq", "q"]
            self.recorder = csv.DictWriter(csvfile, fieldnames=fieldnames)
            self.recorder.writeheader()

    @staticmethod
    def record_csv_file_task(self: "MotorManager"):
        if not hasattr(self, "recorder") or self.recorder is None:
            return
        timestamp = int(time.time() * 1000)
        notify_data = self.motor_data
        for name, data in notify_data.items():
            self.recorder.writerow(
                {
                    "timestamp": timestamp,
                    "motor_name": name,
                    "tau": data.tau if data else 0.0,
                    "dq": data.dq if data else 0.0,
                    "q": data.q if data else 0.0,
                }
            )

    def run(self):
        self.loop_flag = True
        self.transfer_thread = threading.Thread(target=self.loop)
        self.transfer_thread.start()

    def stop(self):
        self.stop_without_join()
        if self.transfer_thread is not None:
            self.transfer_thread.join()
        # 由于我们使用的是multiprocessing.Pool，需要在析构中close/join
        # 这里可以再补充:
        self.pool.close()
        self.pool.join()

    def stop_without_join(self):
        self.loop_flag = False

    def loop(self):
        cur_time = time.time() * 1000
        next_run_time = cur_time + self.cmd_interval_ms
        while self.loop_flag:
            for task_name, task in self.task_list.items():
                try:
                    task(self)
                except Exception as e:
                    logger.warning(f"run task: {task_name} has trouble: {traceback.format_exc()}")
            cur_time = time.time() * 1000
            time_delta = next_run_time - cur_time

            if time_delta < 0:
                logger.warning(f"loop run too slow, took {self.cmd_interval_ms - time_delta} ms")
                next_run_time = (
                    ((time_delta // self.cmd_interval_ms) * -1 + 1)
                    * self.cmd_interval_ms
                    + next_run_time
                )
            else:
                sleep_seconds = time_delta / 1000
                next_run_time += self.cmd_interval_ms
                time.sleep(sleep_seconds)