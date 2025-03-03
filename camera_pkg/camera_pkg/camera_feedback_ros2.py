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
    3. 發布 - 回傳資訊至ROS2

'''

import socket
import time
import numpy as np

import rclpy
from rclpy.node import Node
from camera_msg_pkg.msg import Camera, CameraData

# ----------------------- 基本參數設定 -----------------------

# 定義 控制 IP 與 Port
DEVICE_IP = "192.168.144.200"
DEVICE_PORT = 2000

# 定義回傳命令 & 格式
rec_cmd = [0x4B, 0x4B, 0x01, 0x97]    # 回傳 'Data' 命令控制
buffer_size = 32                      # 接收 'Data' 位元長度

# ---------------------- 回傳訊息資料格式 ---------------------
class ReceiveMsg:
    def __init__(self):
        self.zAngle = 0.0         # Z軸轉動角
        self.pitchAngle = 0.0     # 俯仰角
        self.rollAngle = 0.0      # 滾動角
        self.yawAngle = 0.0       # 航向角
        self.targetDist = 0.0     # 目標距離 (單位：M)
        self.targetAtt = 0.0      # 目標相對高度 (單位：M)
        self.targetLng = 0.0      # 目標經度
        self.targetLat = 0.0      # 目標緯度
        self.eoZoom = 0.0         # 可見光放大倍數

    # 解析 'KTG-TT30' Data
    def parse(self, buffer, length, out):
        if length<30 or not buffer:
            return False

        try:

            # 雲台角度解析
            out.zAngle=np.frombuffer(buffer[0:2],dtype=np.float16)[0]
            out.pitchAngle=np.frombuffer(buffer[5:7],dtype=np.dtype('<i2'))[0]/100
            out.rollAngle=np.frombuffer(buffer[7:9],dtype=np.dtype('<i2'))[0]/100
            out.yawAngle=np.frombuffer(buffer[9:11],dtype=np.dtype('<i2'))[0]/100

            # 目標測距解析
            out.targetDist=np.frombuffer(buffer[12:14],dtype=np.dtype('<u2'))[0]/10
            out.targetLat=np.frombuffer(buffer[16:20],dtype=np.dtype('<i4'))[0]/10000000
            out.targetLng=np.frombuffer(buffer[20:24],dtype=np.dtype('<i4'))[0]/10000000
            
            return True
        
        except Exception as e:
            print("Error parsing data" ,e)
            return False
        
    def recv_packets(sock, packet_size=32, header=b'\x4B\x4B'):

        # 建立緩衝區
        buffer = bytearray()

        # 接收字元 並加入緩衝區
        while True:                         
            data = sock.recv(1024)
            if not data:
                break 
            buffer.extend(data)

            # 找尋頭幀 (0x4B, 0x4B)
            while len(buffer) >= packet_size:
                start_idx = buffer.find(header)
                if start_idx == -1:
                    buffer = bytearray()
                    break

                # 如果包頭不在緩衝區最前面，捨棄前面的數據
                if start_idx > 0:
                    del buffer[:start_idx]

                # 確認緩衝區中是否有足夠的數據組成一個完整封包
                if len(buffer) < packet_size:
                    break

                # 提取出完整的封包
                packet = buffer[:packet_size]

                # 從緩衝區中刪除已處理的封包數據
                del buffer[:packet_size]

                # 回傳封包
                yield packet

# ---------------------- ROS2 Node 定義 ----------------------
class CameraFeedbackPublisher(Node):
    def __init__(self):
        super().__init__('camera_feedback_publisher_node')
        self.publisher_ = self.create_publisher(Camera, '/camera_data_pub', 10)

# ------------------------ 主要執行序 -------------------------
def main():

    # 出始化 ROS2 系統並建立 Node
    rclpy.init()
    node = CameraFeedbackPublisher()

    # 建立 ReceiveMsg 物件
    msg = ReceiveMsg()

    # 建立 TCP socket 並連線設備
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)      # Create TCP socket
    s.connect((DEVICE_IP, DEVICE_PORT))                        # Connect to device

    try:
        
        # 打開雷射測距
        laser_cmd = (
            b'\x4b\x4b\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x88'  # 固定頭幀
            b'\x01'                                                         # 可見光模式
            b'\x21\x01\x00\x00\x00\x00\x00\x00'                             # CMD 指令部分
            b'\x81\x01'                                                     # CRC 校驗碼
        )
        s.send(laser_cmd)
        print("開啟雷射 - Wait 1 sec")
        time.sleep(1)
        
        # 循環讀取並解析封包
        for packet in ReceiveMsg.recv_packets(s, packet_size=32, header=b'\x4B\x4B'):
            if msg.parse(packet, len(packet), msg):
                # print(f"roll:{msg.rollAngle}, yaw:{msg.yawAngle}, pitch:{msg.pitchAngle}")
                # print(f"測距: {msg.targetDist}")

                # 建立 CameraData 訊息並填入解析數值
                data_msg = CameraData()
                data_msg.rollangle = float(msg.rollAngle)
                data_msg.yawangle = float(msg.yawAngle)
                data_msg.pitchangle = float(msg.pitchAngle)
                data_msg.targetdist = float(msg.targetDist)

                # 建立 Camera 訊息，並將 data_msg 放入 data 陣列中
                camera_msg = Camera()
                camera_msg.data = [data_msg]

                # 發布 Camera 訊息至 ROS2 主題
                node.publisher_.publish(camera_msg)
                node.get_logger().info(
                    f"發布ROS2 Camera 資料: roll={data_msg.rollangle}, yaw={data_msg.yawangle},pitch={data_msg.pitchangle} "
                    f"targetdist={data_msg.targetdist}"
                )

            else:
                print("Can't Received The Data")

            time.sleep(0.1) # 10Hz
            
    except KeyboardInterrupt:
        print("\n!!使用者中斷程式執行!!")
    except Exception as e:
        print("發生例外:", e)
    finally:
        s.close()

if __name__ == "__main__":
    main()
