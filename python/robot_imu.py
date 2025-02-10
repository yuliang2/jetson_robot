# coding:UTF-8
"""
    机器人读取姿态的程序
"""
import time
import datetime
import platform
import struct
import sys
sys.path.append('../wit_protocol')

import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

def readConfig(device):
    """
    读取配置信息示例    Example of reading configuration information
    :param device: 设备模型 Device model
    :return:
    """
    tVals = device.readReg(0x02,3)  #读取数据内容、回传速率、通讯速率   Read data content, return rate, communication rate
    if (len(tVals)>0):
        print("返回结果：" + str(tVals))
    else:
        print("无返回")
    tVals = device.readReg(0x23,2)  #读取安装方向、算法  Read the installation direction and algorithm
    if (len(tVals)>0):
        print("返回结果：" + str(tVals))
    else:
        print("无返回")

def onUpdate(deviceModel):
    """
    数据更新事件  Data update event
    :param deviceModel: 设备模型    Device model
    :return:
    """
    print("\r加速度：{:8.4f},{:8.4f},{:8.4f}".format(deviceModel.getDeviceData("accX"), deviceModel.getDeviceData("accY"), deviceModel.getDeviceData("accZ")), end="\t")
    print("角速度：{:8.3f},{:8.3f},{:8.3f}".format(deviceModel.getDeviceData("gyroX"), deviceModel.getDeviceData("gyroY"), deviceModel.getDeviceData("gyroZ")), end="\t")
    print("角度：{:8.3f},{:8.3f},{:8.3f}".format(deviceModel.getDeviceData("angleX"), deviceModel.getDeviceData("angleY"), deviceModel.getDeviceData("angleZ")), end="\t")
    print("四元数：{:8.5f},{:8.5f},{:8.5f}".format(deviceModel.getDeviceData("q1"), deviceModel.getDeviceData("q2"), deviceModel.getDeviceData("q3"), deviceModel.getDeviceData("q4")), end='')


if __name__ == '__main__':
    device = deviceModel.DeviceModel(
        "我的WT901",
        WitProtocolResolver(),
        JY901SDataProcessor(),
        "51_0"
    )

    if (platform.system().lower() == 'linux'):
        device.serialConfig.portName = "/dev/ttyTHS1"   #设置串口   Set serial port
    else:
        device.serialConfig.portName = "COM9"          #设置串口   Set serial port
    device.serialConfig.baud = 115200                     #设置波特率  Set baud rate
    device.openDevice()                                 #打开串口   Open serial port
    readConfig(device)                                  #读取配置信息 Read configuration information
    device.dataProcessor.onVarChanged.append(onUpdate)  #数据更新事件 Data update event

    input()
    device.closeDevice()
