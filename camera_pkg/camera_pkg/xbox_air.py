#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File   : camera_gui_ros2.py
Author : LYX(先驅), FantasyWilly
Email  : FantasyWilly - bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 : 創建簡單 GUI 界面, 用於發送相機控制指令, 同時接收相機回傳資料發布至 ROS2
"""

# Python
import time
import threading
import socketserver

# ROS2
import rclpy
from rclpy.node import Node

# ROS2 自定義
import camera_pkg.camera_loop_command as loop_cm
from camera_pkg.camera_communication import CommunicationController
from camera_pkg.camera_decoder import ReceiveMsg
from camera_msg_pkg.msg import Camera, CameraData

# ---------- 基本參數(全域參數) ----------
CAMERA_IP = "192.168.144.200"       # 相機控制 IP
CAMERA_PORT = 2000                  # 相機控制 Port

PROXY_LISTEN_IP = "192.168.0.230"   # 代理服務監聽的 IP
PROXY_LISTEN_PORT = 9999            # 代理服務監聽的埠號

# 建立全域的 controller 與解析器物件
controller = CommunicationController(CAMERA_IP, CAMERA_PORT)
gimbal_msg = ReceiveMsg()

# ----------------------- [CameraFeedbackPublisher] 初始化 ROS2 Node 與發布者 -----------------------
class CameraFeedbackPublisher(Node):
    def __init__(self):
        super().__init__('xbox_air_node')
        self.declare_parameter('gimbal_step', 50)
        self.declare_parameter('zoom_duration', 0.3)
        self.declare_parameter('photo_continous_count', 3)

        self.gimbal_step = self.get_parameter('gimbal_step').get_parameter_value().integer_value
        self.zoom_duration = self.get_parameter('zoom_duration').get_parameter_value().double_value
        self.photo_continous_count = self.get_parameter('photo_continous_count').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Camera, '/camera_data_pub', 10)

# ----------------------- [BackgroundManager] 背景線程管理 -----------------------
class BackgroundManager:
    def __init__(self):
        self.controller = controller
        self.gimbal_msg = gimbal_msg
        rclpy.init()
        self.ros_node = CameraFeedbackPublisher()
        self.stop_event = threading.Event()
        self.threads = []

    def start(self):
        # (1) 建立與相機的連線
        self.controller.connect()
        time.sleep(1)

        # (2) 啟動 loop_in_background 線程（持續發送空命令）
        self.loop_thread = threading.Thread(
            target=loop_cm.loop_in_background,
            args=(self.controller, self.stop_event),
            daemon=True
        )
        self.loop_thread.start()
        self.threads.append(self.loop_thread)
        print("[開啟]: LOOP-CMD (等待 1 秒)")
        time.sleep(1)

        # (3) 啟動 ROS2 的 spin 線程
        self.ros_spin_thread = threading.Thread(
            target=lambda: rclpy.spin(self.ros_node),
            daemon=True
        )
        self.ros_spin_thread.start()
        self.threads.append(self.ros_spin_thread)
        print("[ROS2]: 節點啟動 (等待 1 秒)")
        time.sleep(1)

        # (4) 啟動背景接收並發布資料的線程
        self.ros_publish_thread = threading.Thread(
            target=self.receive_and_publish,
            daemon=True
        )
        self.ros_publish_thread.start()
        self.threads.append(self.ros_publish_thread)
        print("[背景線程] 開始接收並發布相機資料到 ROS2")

    def receive_and_publish(self):
        # 持續接收相機回傳封包，解析後發布至 ROS2
        for packet in CommunicationController.recv_packets(self.controller.sock, packet_size=32, header=b'\x4B\x4B'):
            if self.stop_event.is_set():
                break
            if self.gimbal_msg.parse(packet, len(packet), self.gimbal_msg):
                data_msg = CameraData()
                data_msg.rollangle = float(self.gimbal_msg.rollAngle)
                data_msg.yawangle = float(self.gimbal_msg.yawAngle)
                data_msg.pitchangle = float(self.gimbal_msg.pitchAngle)
                
                target_distance = float(self.gimbal_msg.targetDist)
                if target_distance > 1500:
                    continue
                data_msg.targetdist = target_distance

                camera_msg = Camera()
                camera_msg.data = [data_msg]
                self.ros_node.publisher_.publish(camera_msg)
            else:
                print("無法解析資料")
            if self.stop_event.is_set():
                break
            time.sleep(0.05)
    
    def shutdown(self):
        self.stop_event.set()
        for t in self.threads:
            t.join()
        self.controller.disconnect()
        rclpy.shutdown()

# -------------------- [forward_command_to_camera] 利用 controller 的連線發送命令 --------------------
def forward_command_to_camera(command_data: bytes) -> bytes:
    global controller
    try:
        with controller.lock:
            controller.sock.sendall(command_data)
            response = controller.sock.recv(1024)
            return response
    except Exception as e:
        print(f"[代理] forward_command_to_camera 發生錯誤：{e}")
        return b''

# -------------------- [ProxyHandler] 處理地面端連線 --------------------
class ProxyHandler(socketserver.BaseRequestHandler):
    def handle(self):
        print(f"[代理] 接收到來自 {self.client_address} 的連線")
        while True:
            try:
                data = self.request.recv(1024)
                if not data:
                    print(f"[代理] 客戶端 {self.client_address} 已關閉連線")
                    break
                print(f"[代理] 從地面端接收到命令：{data.hex().upper()}")
                response = forward_command_to_camera(data)
                if response:
                    self.request.sendall(response)
                    print(f"[代理] 已將回應發送給地面端：{response.hex().upper()}")
                else:
                    print("[代理] 未獲得有效回應")
            except Exception as e:
                print(f"[代理] 處理連線錯誤：{e}")
                break

# -------------------- [ThreadedTCPServer] TCP代理 相關設定 --------------------
class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True

def start_proxy_server(host: str, port: int) -> ThreadedTCPServer:
    server = ThreadedTCPServer((host, port), ProxyHandler)
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    print(f"[代理] 代理服務器已啟動，監聽 {host}:{port}")
    return server

# ----------------------- [main] 主要執行序 -----------------------
def main():
    global controller  # 確保使用全域 controller
    global ros2_publisher

    try:
        background_manager = BackgroundManager()
        background_manager.start()
    except Exception as e:
        print("初始化錯誤:", e)
        return

    # 啟動 TCP 代理服務器，讓地面端連線後可持續發送命令
    proxy_server = start_proxy_server(PROXY_LISTEN_IP, PROXY_LISTEN_PORT)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("使用者中斷程式執行")
    except Exception as e:
        print("發生例外:", e)
    finally:
        background_manager.shutdown()
        proxy_server.shutdown()
        proxy_server.server_close()

if __name__ == "__main__":
    main()
