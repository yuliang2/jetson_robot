# -*- coding: utf-8 -*-
# @File : com_with_mcu.py
# @brief: jetson用于和mcu通信的程序
# @note : 通信格式< 12A 34B 56C 78D 90E 12F 34G 56H CRC>\r\n 其中A-H表示八个舵机，CRC为2位0-F的ASCII码，注意H后有且仅有一个空格

# TODO: 修改command格式，以便于和仿真环境更好地对接

from multiprocessing import Process, Queue
import serial
import re
import time


def log(msg):
    print(msg)
    if log_q and log_q is not None:
        log_q.put(msg)

def calculate_crc(data):
    """计算CRC8校验值"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
        crc &= 0xFF
    return crc

class Com(Process):

    def __init__(self, port=None, status_qs=None, command_q=None, output=False, log_q=None):
        Process.__init__(self)

        self.port = port
        self.status_qs = status_qs
        self.command_q = command_q
        self.output = output
        self.log_q = log_q

    def command_decode(self, s):
        pattern = r'<\s*(-?\d+)A\s*(-?\d+)B\s*(-?\d+)C\s*(-?\d+)D\s*(-?\d+)E\s*(-?\d+)F\s*(-?\d+)G\s*(-?\d+)H\s*([\da-fA-F]{2})\s*>'
        matches = re.findall(pattern, s)
        if matches == []:
            return None
        values = matches[0][:-1]
        crc_received = int(matches[0][-1], 16)

        keys = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']
        results = {}
        for k, v in zip(keys, values):
            results[k] = int(v)

        # 计算CRC以验证接收到的数据
        crc_calculated = calculate_crc(s[:-3].encode())  # 去掉最后的CRC和'>'
        if crc_calculated != crc_received:
            log("[×] CRC校验失败")
            return None

        return results

    def command_encode(self, command):
        s = '<'
        keys = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']
        for k, v in command.items():
            if k not in keys:
                continue
            s += str(v) + k
        crc = calculate_crc(s.encode())
        return f"{s}{crc:02X}>\r\n"

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
            else:
                s = b"<>00>\r\n"  # 发送空命令，CRC 设为 00

            if self.output:
                log(s)
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


class PosPublish(Process):
    def __init__(self, command_q=None, output=False, log_q=None):
        Process.__init__(self)
        self.command_q = command_q
        self.output = output
        self.log_q = log_q

    def run(self):
        global log_q
        log_q = self.log_q
        log("位置发布进程启动")

        while True:
            self.command_q.put({'A': 2266, 'B': 1724, 'C': 3800, 'D': 1519, 'E': 2178, 'F': 1227, 'G': 2369, 'H': 1973})
            # self.command_q.put({'A': 2266, 'B': 1824, 'C': 2024, 'D': 1619, 'E': 2278, 'F': 1327, 'G': 630, 'H': 2073})
            time.sleep(1)
            self.command_q.put({'A': 2166, 'B': 1724, 'C': 3800, 'D': 1519, 'E': 2078, 'F': 1227, 'G': 2369, 'H': 1973})
            # self.command_q.put({'A': 2066, 'B': 1624, 'C': 3700, 'D': 1419, 'E': 2078, 'F': 1127, 'G': 2269, 'H': 1873})
            time.sleep(1)

if __name__ == "__main__":
    command_q = Queue(maxsize=10)
    command_q.put({'A': 2166, 'B': 1724, 'C': 3800, 'D': 1519, 'E': 2178, 'F': 1227, 'G': 2369, 'H': 1973})
    com = Com(port="/dev/my485serial_mcu", command_q=command_q, output=True)
    pos_publish = PosPublish(command_q=command_q, output=True)
    com.start()
    pos_publish.start()
    com.join()
    pos_publish.join()
