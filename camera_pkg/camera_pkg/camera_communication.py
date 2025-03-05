#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File   : socket_communication.py
author : FantasyWilly
email  : bc697522h04@gmail.com

檔案大綱 : 
    處理與相機的網路連線、發送命令與解析回傳資料
"""

import socket
import threading

class CommunicationController:
    def __init__(self, ip: str, port: int, timeout: float = 5.0):
        """
        初始化 CommunicationController

        Args:
            ip (str)                  : 目標主機 IP
            port (int)                : 目標主機 Port
            timeout (float, optional) : Socket 超時時間 (預設 5 秒)
        """
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)

        # 確保一次只能有一個執行緒在呼叫 send_command()
        self.lock = threading.Lock()

    # 啟用TCP連線
    def connect(self) -> None:
        self.sock.connect((self.ip, self.port))
        print(f"已連接到 GCU: {self.ip}:{self.port}")

    # 關閉TCP連線
    def disconnect(self) -> None:
        self.sock.close()
        print("連接已關閉")

    # 單一命令發送 (CMD)
    def send_command(self, cmd_bytes):
        try:
            self.sock.sendall(cmd_bytes)
            print("[發送]: 成功")
            print("-----------------------------")
        except socket.error as e:
            print("[發送]: !!失敗!!")
            print("[錯誤訊息]:", e)
            print("-----------------------------")

    # 連續命令發送 (CMD)
    def loop_send_command(self, loop_cmd_bytes):
        try:
            self.sock.sendall(loop_cmd_bytes)

        except socket.error as e:
            print("[發送]: !!失敗!!")
            print("[錯誤訊息]:", e)
            print("-----------------------------")

    @staticmethod
    def recv_packets(sock, packet_size=32, header=b'\x4B\x4B'):
        """
        從 socket 中接收資料並提取完整封包:
            :param sock: 已連線的 socket 物件
            :param packet_size: 每個封包的位元組數
            :param header: 封包起始的標識位元
        """
        
        # 建立資料緩衝區
        buffer = bytearray()  
        while True:
            data = sock.recv(256)
            if not data:
                break
            buffer.extend(data)

            # 當緩衝區內資料足夠一個封包時，開始尋找封包頭
            while len(buffer) >= packet_size:
                start_idx = buffer.find(header)
                if start_idx == -1:
                    buffer = bytearray()
                    break
                
                # 刪除封包頭前的雜訊資料
                if start_idx > 0:
                    del buffer[:start_idx]  
                
                # 資料不足完整封包則等待更多資料
                if len(buffer) < packet_size:
                    break

                # 提取出一個完整封包
                packet = buffer[:packet_size]
                del buffer[:packet_size]
                yield packet         
