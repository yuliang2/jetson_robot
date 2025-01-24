import logging.config
import time
import threading
import traceback
import random
import os
import csv
from datetime import datetime
import logging
import yaml
import typing
import copy
import enum
import concurrent

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
    worker_thread: threading.Thread | None = None
    transfer_thread_pool: concurrent.futures.ThreadPoolExecutor | None = None
    cmd_interval_ms: int
    motor_dict: dict[str, MotorInstance] = dict()
    motor_group_dict: dict[str, list[MotorInstance]] = dict()
    loop_flag: bool = False

    task_list: dict[str, typing.Callable] = dict()

    motor_notify_callback_list: list[typing.Callable] = list()

    def __init__(self, cmd_interval_ms: int):
        self.cmd_interval_ms = cmd_interval_ms
        self.register_task(MotorManager.transfer_motor_cmds_task, task_name="TransferCmds")
        self.register_task(MotorManager.notify_motor_data_task, task_name="NotifyData")
        # self.init_record_csv()
        # self.register_task(MotorManager.record_csv_file_task, task_name="RecordCSV")

    def __del__(self):
        pass

    def register_motor(self, motor: MotorInstance):
        name = motor.get_motor_name()
        self.motor_dict[name] = motor
        motor.reset()
        for name, motor in self.motor_dict.items():
            group_name = name.split("-")[0]
            self.motor_group_dict[group_name] = self.motor_group_dict.get(group_name, list()).append(motor)
        return name

    def get_motor(self, motor_name: str) -> MotorInstance:
        return self.motor_dict.get(motor_name)

    def register_task(self, task: typing.Callable, task_name: str = None) -> str:
        if task_name is None or task_name == "":
            task_name = task.__name__ + "-" + str(time.time * 1000)
        self.task_list[task_name] = task
        return task_name

    @staticmethod
    def transfer_motor_cmds_task(self: "MotorManager"):

        def _inner_transfer_task(motor_list: list[MotorInstance]):
            for motor in motor_list:
                try:
                    motor.execute()
                except Exception as e:
                    logger.warning(f"motor: {motor.get_motor_name()} tranfer error: {traceback.format_exc()}")

        with self.transfer_thread_pool as executor:
            futures = [executor.submit(_inner_transfer_task, motor_list) for motor_list in self.motor_group_dict.values()]
            # wait for execute done
            concurrent.futures.as_completed(futures)

    @staticmethod
    def notify_motor_data_task(self: "MotorManager"):
        for callback in self.motor_notify_callback_list:
            try:
                callback(self.motor_dict)
            except Exception as e:
                logger.warning(f"notify data callback error: {traceback.format_exc()}")

    def add_motor_data_callback(self, callback: typing.Callable):
        self.motor_notify_callback_list.append(callback)

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
        if self.recorder is None:
            return
        timestamp = int(time.time() * 1000)
        for name, motor in self.motor_dict.items():
            self.recorder.writerow(
                {
                    timestamp: timestamp,
                    "motor_name": name,
                    "tau": motor.tau,
                    "dq": motor.dq,
                    "q": motor.q,
                }
            )

    def run(self):
        self.loop_flag = True
        self.transfer_thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=len(self.motor_group_dict))
        self.worker_thread = threading.Thread(target=self.loop)
        self.worker_thread.start()

    def stop(self):
        self.stop_without_join()
        self.worker_thread.join()

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
            sleep_seconds = self.cmd_interval_ms / 1000
            if time_delta < 0:
                logger.warning(f"loop run too slow, took {self.cmd_interval_ms - time_delta} ms")
                next_run_time = (((time_delta // self.cmd_interval_ms) * -1) + 1) * self.cmd_interval_ms + next_run_time
                # no sleep and run next loop immediately
            else:
                sleep_seconds = (next_run_time - cur_time) / 1000
                next_run_time = next_run_time + self.cmd_interval_ms
                time.sleep(sleep_seconds)
