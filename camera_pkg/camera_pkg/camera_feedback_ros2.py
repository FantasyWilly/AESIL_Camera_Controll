#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
File   : camera_feedback_ros2.py
author : LYX(先驅), FantasyWilly
email  : FantasyWilly - bc697522h04@gmail.com
'''

'''
檔案大綱 :
    1. 接收 - 相機＆雲台回傳
    2. 打開 - 雷射測距
    3. 持續傳送回傳指令 並發布回傳資訊至ROS2
'''

import socket
import time
import numpy as np
import threading    # ** 匯入 threading 模組以建立獨立執行緒 **

import rclpy
from rclpy.node import Node
from camera_msg_pkg.msg import Camera, CameraData

# ----------------------- 基本參數設定 -----------------------

# ** 定義設備 IP 與 Port **
DEVICE_IP = "192.168.144.200"         # ** 設定設備 IP **
DEVICE_PORT = 2000                    # ** 設定設備 Port **

# ** 定義回傳命令與緩衝區大小 **
rec_cmd = [0x4B, 0x4B, 0x01, 0x97]      # ** 回傳 'Data' 命令控制 **
buffer_size = 32                      # ** 接收 'Data' 資料長度 **

# ---------------------- 回傳訊息資料格式 ---------------------
class ReceiveMsg:
    def __init__(self):
        self.zAngle = 0.0         # ** Z軸轉動角 **
        self.pitchAngle = 0.0     # ** 俯仰角 **
        self.rollAngle = 0.0      # ** 滾動角 **
        self.yawAngle = 0.0       # ** 航向角 **
        self.targetDist = 0.0     # ** 目標距離 (單位：M) **
        self.targetAtt = 0.0      # ** 目標相對高度 (單位：M) **
        self.targetLng = 0.0      # ** 目標經度 **
        self.targetLat = 0.0      # ** 目標緯度 **
        self.eoZoom = 0.0         # ** 可見光放大倍數 **

    # ** 解析 'KTG-TT30' 回傳資料 **
    def parse(self, buffer, length, out):
        if length < 30 or not buffer:
            return False

        try:
            # ** 解析雲台角度數據 **
            out.zAngle = np.frombuffer(buffer[0:2], dtype=np.float16)[0]
            out.pitchAngle = np.frombuffer(buffer[5:7], dtype=np.dtype('<i2'))[0] / 100
            out.rollAngle = np.frombuffer(buffer[7:9], dtype=np.dtype('<i2'))[0] / 100
            out.yawAngle = np.frombuffer(buffer[9:11], dtype=np.dtype('<i2'))[0] / 100

            # ** 解析目標測距與經緯度數據 **
            out.targetDist = np.frombuffer(buffer[12:14], dtype=np.dtype('<u2'))[0] / 10
            out.targetLat = np.frombuffer(buffer[16:20], dtype=np.dtype('<i4'))[0] / 10000000
            out.targetLng = np.frombuffer(buffer[20:24], dtype=np.dtype('<i4'))[0] / 10000000

            return True

        except Exception as e:
            print("Error parsing data", e)
            return False

    # ** 從 socket 中接收封包，並以生成器形式回傳完整封包 **
    def recv_packets(sock, packet_size=32, header=b'\x4B\x4B'):
        buffer = bytearray()  # ** 建立接收資料緩衝區 **
        while True:
            data = sock.recv(2048)  # ** 從 socket 接收最多 2048 位元的資料 **
            if not data:
                break
            buffer.extend(data)  # ** 將接收的資料加入緩衝區 **

            # ** 當緩衝區資料足夠時，開始搜尋完整封包 **
            while len(buffer) >= packet_size:
                start_idx = buffer.find(header)  # ** 搜尋封包起始頭 (0x4B, 0x4B) **
                if start_idx == -1:
                    buffer = bytearray()  # ** 若找不到起始頭，則清空緩衝區 **
                    break

                # ** 如果封包起始頭不在緩衝區最前面，刪除前面多餘資料 **
                if start_idx > 0:
                    del buffer[:start_idx]

                if len(buffer) < packet_size:
                    break  # ** 若不足一個完整封包，則等待更多資料 **

                packet = buffer[:packet_size]  # ** 提取一個完整封包 **
                del buffer[:packet_size]         # ** 從緩衝區中刪除已處理的資料 **
                yield packet  # ** 回傳該封包給呼叫端 **

# ---------------------- ROS2 Node 定義 ----------------------
class CameraFeedbackPublisher(Node):
    def __init__(self):
        super().__init__('camera_feedback_publisher_node')  # ** 初始化 ROS2 節點，節點名稱為 'camera_feedback_publisher_node' **
        self.publisher_ = self.create_publisher(Camera, '/camera_data_pub', 10)  # ** 建立 Publisher，將 Camera 訊息發布到 '/camera_data_pub' **

# ------------------------ 主要執行序 -------------------------
# ** 修改傳送回傳指令函數，加入參數 sock **
def send_rec_cmd(sock):
    while True:
        try:
            # ** 將 rec_cmd 轉換成 bytes 後傳送 **
            sock.send(bytes(rec_cmd))
        except Exception as e:
            print("傳送指令時發生例外:", e)
            break
        time.sleep(0.1)  # ** 每 0.1 秒傳送一次指令 (約 10Hz) **

def main():
    # ** 初始化 ROS2 系統並建立節點 **
    rclpy.init()
    node = CameraFeedbackPublisher()

    # ** 建立接收訊息物件 **
    msg = ReceiveMsg()

    # ** 建立 TCP socket 並連線到設備 **
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # ** 建立 TCP socket **
    s.connect((DEVICE_IP, DEVICE_PORT))                    # ** 連線到指定設備 IP 與 Port **

    try:
        # ** 開啟雷射測距：先送出一次開啟雷射指令 **
        laser_cmd = (
            b'\x4b\x4b\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x88'  # ** 固定頭幀 **
            b'\x01'                                                         # ** 可見光模式 **
            b'\x21\x01\x00\x00\x00\x00\x00\x00'                             # ** CMD 指令部分 **
            b'\x81\x01'                                                     # ** CRC 校驗碼 **
        )
        s.send(laser_cmd)  # ** 傳送開啟雷射指令 **
        print("開啟雷射 - Wait 1 sec")
        time.sleep(1)  # ** 等待 1 秒讓雷射測距啟動 **

        # ** 建立傳送回傳指令的獨立執行緒，並將 socket s 傳入 **
        sender_thread = threading.Thread(target=send_rec_cmd, args=(s,), daemon=True)
        sender_thread.start()

        # ** 持續接收並解析封包，並發布至 ROS2 主題 **
        for packet in ReceiveMsg.recv_packets(s, packet_size=32, header=b'\x4B\x4B'):
            if msg.parse(packet, len(packet), msg):
                # ** 建立 CameraData 訊息並填入解析數值 **
                data_msg = CameraData()
                data_msg.rollangle = float(msg.rollAngle)
                data_msg.yawangle = float(msg.yawAngle)
                data_msg.pitchangle = float(msg.pitchAngle)
                data_msg.targetdist = float(msg.targetDist)

                # ** 建立 Camera 訊息，並將 data_msg 放入 data 陣列中 **
                camera_msg = Camera()
                camera_msg.data = [data_msg]

                # ** 發布 Camera 訊息至 ROS2 主題 '/camera_data_pub' **
                node.publisher_.publish(camera_msg)
                node.get_logger().info(
                    f"發布ROS2 Camera 資料: roll={data_msg.rollangle}, yaw={data_msg.yawangle}, "
                    f"pitch={data_msg.pitchangle}, targetdist={data_msg.targetdist}"
                )
            else:
                print("Can't Received The Data")

            time.sleep(0.05)  # ** 控制接收頻率，約 20Hz **

    except KeyboardInterrupt:
        print("\n!!使用者中斷程式執行!!")
    except Exception as e:
        print("發生例外:", e)
    finally:
        s.close()  # ** 關閉 socket 連線 **

if __name__ == "__main__":
    main()
