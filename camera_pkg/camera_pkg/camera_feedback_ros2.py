#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
File   : camera_feedback_ros2.py
author : LYX(先驅), FantasyWilly
email  : FantasyWilly - bc697522h04@gmail.com

說明  : 接收雲台資訊並發布至 ROS2
         透過 communication 模組處理相機資料解析與封包接收
'''

import socket
import time
import threading

import rclpy
from rclpy.node import Node
from camera_msg_pkg.msg import Camera, CameraData

import camera_pkg.socket_communication as  communication
import camera_pkg.camera_decoder as decoder

# 定義設備 IP 與 Port，直接使用 communication 模組中的設定
DEVICE_IP = communication.TCP_IP        # 相機 IP
DEVICE_PORT = communication.TCP_PORT    # 相機 Port

# 回傳命令位元組定義
rec_cmd = [0x4B, 0x4B, 0x01, 0x97]

class CameraFeedbackPublisher(Node):
    def __init__(self):
        super().__init__('camera_feedback_publisher_node')
        self.publisher_ = self.create_publisher(Camera, '/camera_data_pub', 10)

def send_rec_cmd(sock):
    """
    持續傳送回傳指令給相機
    :param sock: 已連線的 socket 物件
    """
    while True:
        try:
            sock.send(bytes(rec_cmd))
        except Exception as e:
            print("傳送指令時發生例外:", e)
            break
        time.sleep(0.05)

def main():
    rclpy.init()
    node = CameraFeedbackPublisher()

    # 建立解析回傳資料的物件，使用 communication 模組中的 ReceiveMsg
    msg = decoder.ReceiveMsg()

    # 建立 TCP socket 並連線到相機
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((DEVICE_IP, DEVICE_PORT))

    try:
        # 開啟雷射測距，送出一次開啟雷射的指令
        laser_cmd = (
            b'\x4b\x4b\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x88'      # 固定頭幀
            b'\x01'                                                          # 可見光模式
            b'\x21\x01\x00\x00\x00\x00\x00\x00'                              # CMD 指令部分
            b'\x81\x01'                                                      # CRC 校驗碼
        )
        s.send(laser_cmd)
        print("開啟雷射 - 等待 1 秒")
        time.sleep(1)

        # 建立獨立執行緒持續傳送回傳指令，並將 socket 傳入
        sender_thread = threading.Thread(target=send_rec_cmd, args=(s,), daemon=True)
        sender_thread.start()

        # 持續接收並解析來自相機的封包
        for packet in decoder.ReceiveMsg.recv_packets(s, packet_size=32, header=b'\x4B\x4B'):
            if msg.parse(packet, len(packet), msg):
                data_msg = CameraData()
                data_msg.rollangle = float(msg.rollAngle)
                data_msg.yawangle = float(msg.yawAngle)
                data_msg.pitchangle = float(msg.pitchAngle)
                data_msg.targetdist = float(msg.targetDist)

                # 建立 Camera 訊息，並將 data_msg 放入 data 陣列中
                camera_msg = Camera()
                camera_msg.data = [data_msg]

                # 發布 Camera 訊息至 ROS2 主題 '/camera_data_pub'
                node.publisher_.publish(camera_msg)
                node.get_logger().info(
                    f"[發布] Camera 資料: roll={data_msg.rollangle}, yaw={data_msg.yawangle}, "
                    f"pitch={data_msg.pitchangle}, targetdist={data_msg.targetdist}"
                )
            else:
                print("無法解析資料")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n!!使用者中斷程式執行!!")
    except Exception as e:
        print("發生例外:", e)
    finally:
        s.close()

if __name__ == "__main__":
    main()
