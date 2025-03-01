#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''

File   : camera_feedback.py

author : LYX(先驅), FantasyWilly
email  : bc697522h04@gmail.com

'''

'''

檔案大綱 :
    1. 接收 - 相機＆雲台回傳
    2. 打開 - 雷射測距
    3. 發布 - 回傳資訊至 ROS2

'''

import socket
import time
import numpy as np

import camera_command as cm

# 定義 控制 IP 與 Port
DEVICE_IP = "192.168.144.200"
DEVICE_PORT = 2000

# 定義回傳命令 & 格式
rec_cmd = [0x4B, 0x4B, 0x01, 0x97]    # 回傳 'Data' 命令控制
buffer_size = 32                      # 接收 'Data' 位元長度

# 回傳訊息資料格式
class ReceiveMsg:
    def __init__(self):
        self.zAngle = 0.0         # Z軸轉動角
        self.pitchAngle = 0.0     # 俯仰角
        self.rollAngle = 0.0      # 滾動角
        self.yawAngle = 0.0       # 航向角
        self.isRanged = False     # 測距結果
        self.targetDist = 0.0     # 目標距離 (單位：M)
        self.targetAtt = 0.0      # 目標相對高度 (單位：M)
        self.targetLng = 0.0      # 目標經度
        self.targetLat = 0.0      # 目標緯度
        self.gimbalBit = False    # 雲台開機自檢
        self.eoZoom = 0.0         # 可見光放大倍數

    # 解析 'KTG-TT30' Data
    def parse(self, buffer, length, out):
        if length<30 or not buffer:
            return False
        
        #print("Received data:",buffer)

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

# ------------------------------------------------主要執行序-------------------------------------------------------------------
    
# 定義 ReceiveMsg 的 Class
msg = ReceiveMsg()

# 定義 連線
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)      # Create TCP socket
s.connect((DEVICE_IP, DEVICE_PORT))                        # Connect to device

def main():
    try:

        # 在程式啟動時，只發布一次雷射測距控制命令 (此處 1 表示開啟)**
        laser_cmd_bytes = cm.Command.laser_command(1)
        s.send(laser_cmd_bytes)
        print("開啟雷射 - Wait 1 sec")
        time.sleep(1)
        
        # 循環讀取並解析封包
        for packet in ReceiveMsg.recv_packets(s, packet_size=32, header=b'\x4B\x4B'):
            if msg.parse(packet, len(packet), msg):
                print(f"roll:{msg.rollAngle}, yaw:{msg.yawAngle}, pitch:{msg.pitchAngle}")
                print(f"測距: {msg.targetDist}")
            else:
                print("Can't Received The Data")

            time.sleep(0.1) # 10Hz
            
    except KeyboardInterrupt:
        print("使用者中斷程式執行")
    except Exception as e:
        print("發生例外:", e)
    finally:
        s.close()

if __name__ == "__main__":
    main()
    