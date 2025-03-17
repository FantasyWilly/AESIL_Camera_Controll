#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TCP 代理服務示例
================
功能：
  1. 在 Orin Nano 上與相機建立單一 TCP 連線。
  2. 啟動一個 TCP 代理服務，監聽地面端連線（例如埠 9999）。
  3. 收到地面端命令後，透過與相機連線轉發命令、取得回應後回傳給地面端。
  4. 背景中持續發送空命令以讀取相機資料，並透過 ROS2 將資料發布出去。

注意：
  - 此範例僅供參考，實際使用時可能需要進一步處理連線異常、資料同步與多重請求等問題。
  - ROS2 部分假設你已有 GCUPublisher 節點用來發布資料。
"""

# Python
import socket
import threading
import socketserver
import time

# ROS2
import rclpy
from camera_d80_pkg.gcu_ros2_publisher import GCUPublisher
from camera_d80_pkg.camera_protocol import build_packet, send_empty_command
from camera_d80_pkg.camera_decoder import decode_gcu_response

# --- 設定參數 ---
CAMERA_IP = "192.168.168.121"     # 相機 IP
CAMERA_PORT = 2332                # 相機埠號

PROXY_LISTEN_IP = "0.0.0.0"         # 代理服務監聽所有介面
PROXY_LISTEN_PORT = 9999            # 代理服務監聽埠號

# --- 全域變數：與相機的連線與鎖 ---
camera_socket = None
camera_lock = threading.Lock()

# 全域 ROS2 發布者（後續會初始化）
ros2_publisher = None

def connect_to_camera():
    """建立與相機的 TCP 連線"""
    global camera_socket
    camera_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    camera_socket.connect((CAMERA_IP, CAMERA_PORT))
    print(f"已連接到相機：{CAMERA_IP}:{CAMERA_PORT}")

def continuous_empty_command_loop():
    """
    背景執行緒：持續發送空命令讀取相機資料，
    解碼後透過 ROS2 發布（假設封包與解碼依照你的專案實作）
    """
    global camera_socket, camera_lock, ros2_publisher
    while True:
        try:
            with camera_lock:
                # 組建空命令封包，根據你的協議規範
                empty_packet = build_packet(command=0x00, parameters=b'', include_empty_command=True, enable_request=True)
                camera_socket.sendall(empty_packet)
                # 接收回應，假設回應不超過 1024 字節
                response = camera_socket.recv(1024)
            if response:
                parsed = decode_gcu_response(response)
                print(f"[代理] 空命令回應解碼：{parsed}")
                # 若有 ROS2 發布者，發布資料（此處根據你的發布方法調整）
                if ros2_publisher is not None:
                    # 例如: ros2_publisher.publish_data(parsed['roll'], parsed['pitch'], parsed['yaw'])
                    pass
        except Exception as e:
            print(f"[代理] 空命令循環錯誤: {e}")
        time.sleep(1)  # 控制頻率

# --- TCP 代理服務部分 ---
class ProxyHandler(socketserver.BaseRequestHandler):
    """
    處理來自地面端的連線，
    將接收到的命令透過 camera_socket 轉發給相機，
    並將相機回應返回給地面端。
    """
    def handle(self):
        global camera_socket, camera_lock
        print(f"[代理] 接收到來自 {self.client_address} 的連線")
        try:
            data = self.request.recv(1024)
            if not data:
                return
            print(f"[代理] 從地面端接收到命令：{data.hex().upper()}")
            with camera_lock:
                # 轉發命令到相機
                camera_socket.sendall(data)
                response = camera_socket.recv(1024)
            print(f"[代理] 從相機接收到回應：{response.hex().upper()}")
            self.request.sendall(response)
        except Exception as e:
            print(f"[代理] 處理連線錯誤：{e}")

class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    """多執行緒 TCP 代理服務器"""
    allow_reuse_address = True

def start_proxy_server(host, port):
    """啟動 TCP 代理服務器"""
    server = ThreadedTCPServer((host, port), ProxyHandler)
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True  # 當主程式結束時，線程也自動關閉
    server_thread.start()
    print(f"[代理] 代理服務器已啟動，監聽 {host}:{port}")
    return server

# --- 主程式 ---
if __name__ == '__main__':
    # 初始化 ROS2 節點與發布者
    rclpy.init()
    ros2_publisher = GCUPublisher()
    ros2_thread = threading.Thread(target=rclpy.spin, args=(ros2_publisher,), daemon=True)
    ros2_thread.start()

    # 先與相機建立連線
    connect_to_camera()

    # 啟動背景執行緒，持續發送空命令並發布相機資料
    empty_thread = threading.Thread(target=continuous_empty_command_loop, daemon=True)
    empty_thread.start()

    # 啟動代理服務器，讓地面端連線到 Orin Nano（代理服務）
    proxy_server = start_proxy_server(PROXY_LISTEN_IP, PROXY_LISTEN_PORT)

    # 主執行緒保持運行
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("關閉代理服務器與退出")
        proxy_server.shutdown()
        proxy_server.server_close()
        rclpy.shutdown()
