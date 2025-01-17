# -*- coding: utf-8 -*-
# @File : com_with_mcu.py
# @brief: jetson用于和mcu通信的程序
# @note : 通信格式< 12A 34B 56C 78D 90E 12F 34G 56H >\r\n 其中A-H表示八个舵机

# TODO: 修改串口通信格式以便更高效稳定地和MCU传输数据
# TODO: 修改command格式，以便于和仿真环境更好地对接

from multiprocessing import Process, Queue
import serial
import re
import time


def log(msg):
    print(msg)
    if log_q != None:
        log_q.put(msg)


class Com(Process):

    def __init__(self, port=None, status_qs=None, command_q=None, output=False, log_q=None):
        Process.__init__(self)

        self.port = port
        self.status_qs = status_qs
        self.command_q = command_q
        self.output = output

        self.log_q = log_q

    def command_decode(self, s):
        pattern = r'<\s*(-?\d+)A\s*(-?\d+)B\s*(-?\d+)C\s*(-?\d+)D\s*(-?\d+)E\s*(-?\d+)F\s*(-?\d+)G\s*(-?\d+)H>'
        matches = re.findall(pattern, s)
        if matches == []:
            return None
        values = matches[0]
        keys = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']
        results = {}
        for k, v in zip(keys, values):
            results[k] = int(v)
        return results

    def command_encode(self, command):
        s = '<'
        keys = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']
        for k, v in command.items():
            if k not in keys:
                continue
            else:
                s += str(v) + k
        return s + '>\r\n'

    def task(self):
        if self.port is None:
            return

        try:
            ser = serial.Serial(self.port, 115200)
        except:
            return
        log(f"[√] 串口{self.port}开启成功")

        while True:
            try:
                in_waiting = ser.in_waiting
            except:
                log(f"[×] 串口{self.port}断开连接")
                break
            if in_waiting > 0:
                try:
                    raw_line = ser.readline()
                except:
                    log(f"[×] 串口{self.port}读取失败")
                    break
                try:  # 忽略utf-8解码失败的情况
                    s = raw_line.decode().strip()
                except:
                    continue
                status = self.command_decode(s)
                if status:
                    if self.status_qs:
                        for status_q in self.status_qs:
                            if not status_q.full():
                                status_q.put(status)
                    if self.output:
                        log(status)
            if self.command_q and not self.command_q.empty():
                command = self.command_q.get()
                s = self.command_encode(command).encode()
                ser.write(s)
                ser.flush()

            time.sleep(0.1)

        ser.close()

    def run(self):
        global log_q
        log_q = self.log_q

        log(f"[√] 串口进程启动，使用串口{self.port}")
        while True:
            self.task()
            time.sleep(1)

if __name__ == "__main__":
    command_q = Queue(maxsize=10)
    command_q.put({'A':2048, 'B':2048, 'C':2048, 'D':2048, 'E':2048, 'F':2048, 'G':2048, 'H':2048})
    com = Com(port="COM6", command_q=command_q, output=True)
    com.run()
    # comm.start()
    # comm.join()
