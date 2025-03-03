#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File   : communication.py
author : FantasyWilly
email  : bc697522h04@gmail.com

說明  : 處理與相機的網路連線、發送命令與解析回傳資料
"""

import socket
import numpy as np

# ** 定義相機連線參數 **
TCP_IP = "192.168.144.200"       # **相機 IP**
TCP_PORT = 2000                  # **相機 Port**
BUFFER_SIZE = 32                 # **接收封包位元長度**

def send_command(cmd_bytes):
    """
    **發送命令到相機**
    :param cmd_bytes: 位元組格式的命令
    """
    try:
        # **建立 TCP socket**
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # **連線至相機**
        s.connect((TCP_IP, TCP_PORT))
        # **發送命令**
        s.send(cmd_bytes)
        print(cmd_bytes.hex())
        print("發送成功")
    except socket.error as e:
        print("發送失敗")
        print("錯誤訊息:", e)
    finally:
        s.close()  # **關閉 socket 連線**

class ReceiveMsg:
    """
    **處理相機回傳資料的解析**
    """
    def __init__(self):
        self.zAngle = 0.0         # **Z 軸轉動角**
        self.pitchAngle = 0.0     # **俯仰角**
        self.rollAngle = 0.0      # **滾動角**
        self.yawAngle = 0.0       # **航向角**
        self.targetDist = 0.0     # **目標距離（公尺）**
        self.targetAtt = 0.0      # **目標相對高度**
        self.targetLng = 0.0      # **目標經度**
        self.targetLat = 0.0      # **目標緯度**
        self.eoZoom = 0.0         # **可見光放大倍數**

    def parse(self, buffer, length, out):
        """
        **解析相機回傳的原始資料**
        :param buffer: 原始位元組資料
        :param length: 資料長度
        :param out: 儲存解析結果的 ReceiveMsg 物件
        :return: True 代表解析成功，False 代表失敗
        """
        if length < 30 or not buffer:
            return False

        try:
            # **解析雲台角度**
            out.zAngle = np.frombuffer(buffer[0:2], dtype=np.float16)[0]
            out.pitchAngle = np.frombuffer(buffer[5:7], dtype=np.dtype('<i2'))[0] / 100
            out.rollAngle = np.frombuffer(buffer[7:9], dtype=np.dtype('<i2'))[0] / 100
            out.yawAngle = np.frombuffer(buffer[9:11], dtype=np.dtype('<i2'))[0] / 100

            # **解析目標測距與經緯度**
            out.targetDist = np.frombuffer(buffer[12:14], dtype=np.dtype('<u2'))[0] / 10
            out.targetLat = np.frombuffer(buffer[16:20], dtype=np.dtype('<i4'))[0] / 10000000
            out.targetLng = np.frombuffer(buffer[20:24], dtype=np.dtype('<i4'))[0] / 10000000

            return True
        except Exception as e:
            print("解析資料失敗:", e)
            return False

    @staticmethod
    def recv_packets(sock, packet_size=32, header=b'\x4B\x4B'):
        """
        **從 socket 中接收資料並提取完整封包**
        :param sock: 已連線的 socket 物件
        :param packet_size: 每個封包的位元組數
        :param header: 封包起始的標識位元
        :yield: 每個完整的封包資料
        """
        buffer = bytearray()  # **建立資料緩衝區**
        while True:
            data = sock.recv(1024)
            if not data:
                break
            buffer.extend(data)  # **將接收到的資料加入緩衝區**

            # **檢查緩衝區是否有足夠資料組成完整封包**
            while len(buffer) >= packet_size:
                start_idx = buffer.find(header)
                if start_idx == -1:
                    buffer = bytearray()  # **找不到封包頭則清空緩衝區**
                    break

                if start_idx > 0:
                    del buffer[:start_idx]  # **捨棄封包頭之前的雜訊資料**

                if len(buffer) < packet_size:
                    break  # **若資料不足則等待更多資料**

                # **提取出完整封包**
                packet = buffer[:packet_size]
                del buffer[:packet_size]  # **刪除已處理的封包資料**
                yield packet
