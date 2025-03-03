#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
File   : camera_feedback.py
author : LYX(先驅), FantasyWilly
email  : FantasyWilly - bc697522h04@gmail.com
'''

'''
檔案大綱 :
    1. 接收 - 相機＆雲台回傳
    2. 打開 - 雷射測距
    3. 持續傳送回傳指令以取得數據
'''

import socket
import time
import numpy as np
import threading  # ** 匯入 threading 模組，用來建立傳送指令的執行緒 **

import camera_command as cm

# ** 定義設備 IP 與 Port **
DEVICE_IP = "192.168.144.200"         # ** 設定設備 IP **
DEVICE_PORT = 2000                    # ** 設定設備 Port **

# ** 定義回傳命令與緩衝區大小 **
rec_cmd = [0x4B, 0x4B, 0x01, 0x97]      # ** 回傳 'Data' 命令控制，注意：這裡是持續發送的指令 **
buffer_size = 32                      # ** 接收 'Data' 資料長度 **

# ** 回傳訊息資料格式定義 **
class ReceiveMsg:
    def __init__(self):
        self.zAngle = 0.0         # ** Z軸轉動角 **
        self.pitchAngle = 0.0     # ** 俯仰角 **
        self.rollAngle = 0.0      # ** 滾動角 **
        self.yawAngle = 0.0       # ** 航向角 **
        self.isRanged = False     # ** 測距結果 **
        self.targetDist = 0.0     # ** 目標距離 (單位：M) **
        self.targetAtt = 0.0      # ** 目標相對高度 (單位：M) **
        self.targetLng = 0.0      # ** 目標經度 **
        self.targetLat = 0.0      # ** 目標緯度 **
        self.gimbalBit = False    # ** 雲台開機自檢 **
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
        buffer = bytearray()  # ** 建立資料緩衝區 **
        while True:
            data = sock.recv(1024)  # ** 從 socket 接收資料 **
            if not data:
                break
            buffer.extend(data)  # ** 將接收的資料加入緩衝區 **

            # ** 當緩衝區資料足夠時，開始解析完整封包 **
            while len(buffer) >= packet_size:
                start_idx = buffer.find(header)  # ** 尋找封包起始頭 **
                if start_idx == -1:
                    buffer = bytearray()  # ** 若找不到起始頭，清空緩衝區 **
                    break

                if start_idx > 0:
                    del buffer[:start_idx]  # ** 刪除起始頭之前的無用數據 **

                if len(buffer) < packet_size:
                    break  # ** 若不足一個完整封包，等待更多數據 **

                packet = buffer[:packet_size]  # ** 提取一個完整封包 **
                del buffer[:packet_size]         # ** 從緩衝區中刪除已處理的封包 **
                yield packet  # ** 回傳該封包 **

# ** 全域建立一個 ReceiveMsg 物件，用以儲存解析後的數據 **
msg = ReceiveMsg()

# ** 建立 socket 並連線至設備 **
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # ** 建立 TCP socket **
s.connect((DEVICE_IP, DEVICE_PORT))                    # ** 連線到設備 **

# ** 定義一個函數，用來持續傳送回傳指令 **
def send_rec_cmd():
    while True:
        try:
            # ** 將 rec_cmd 轉換成 bytes 後傳送 **
            s.send(bytes(rec_cmd))
        except Exception as e:
            print("傳送指令時發生例外:", e)
            break
        time.sleep(0.1)  # ** 每 0.1 秒傳送一次指令 (10Hz) **

def main():
    try:
        # 打開雷射測距
        laser_cmd = (
            b'\x4b\x4b\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x88'     # 固定頭幀
            b'\x01'                                                         # 可見光模式
            b'\x21\x01\x00\x00\x00\x00\x00\x00'                             # CMD 指令部分
            b'\x81\x01'                                                     # CRC 校驗碼
        )
        s.send(laser_cmd)
        print("開啟雷射 - Wait 1 sec")
        time.sleep(1)

        # ** 建立傳送指令的執行緒，讓其持續傳送回傳指令 **
        sender_thread = threading.Thread(target=send_rec_cmd, daemon=True)
        sender_thread.start()

        # ** 循環讀取並解析封包 **
        for packet in ReceiveMsg.recv_packets(s, packet_size=32, header=b'\x4B\x4B'):
            if msg.parse(packet, len(packet), msg):
                print(f"roll: {msg.rollAngle}, yaw: {msg.yawAngle}, pitch: {msg.pitchAngle}")
                print(f"測距: {msg.targetDist}")
            else:
                print("Can't Received The Data")
            time.sleep(0.1)  # ** 控制接收頻率，大約 10Hz **
            
    except KeyboardInterrupt:
        print("使用者中斷程式執行")
    except Exception as e:
        print("發生例外:", e)
    finally:
        s.close()

if __name__ == "__main__":
    main()
