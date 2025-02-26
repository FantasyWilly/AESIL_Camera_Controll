#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''

File   : camera_feedback.py

author : FantasyWilly
email  : bc697522h04@gmail.com

'''

'''

1. 接收 - 相機＆雲台回傳
2. 打開 - 雷射測距
3. 發布 - 資訊至 ROS2

'''

import socket
import time
import numpy as np

# Device IP address and port
DEVICE_IP = "192.168.144.200"
DEVICE_PORT = 2000
rec_cmd = [0x4B, 0x4B, 0x01, 0x97]    # 回傳 'Data' 命令控制
buffer_size = 32                  # 接收 'Data' 位元長度

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
    # 效驗碼
    def calculate(self,buffer: list, length: int) -> bytes:
        crc_tmp = 0x0000
        for i in range(length):
            crc_tmp += buffer[i]
        return crc_tmp.to_bytes(1, byteorder='little')
    
# 命令控制
class Command:
    def laser_command(start):#laser(0) 關閉雷射測距 laser(1) 開啟雷射測距
        my_bytes = bytearray(b'\xeb\x90\x0a\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x88\x0f\x21')
        
        my_bytes += bytes([start, 0, 0, 0, 0, 0])        

        crc = CrcTmp.calc(my_bytes)
        my_bytes += crc.to_bytes(2, 'little')
        
        return my_bytes
    
# 校驗碼
class CrcTmp:
    def calc(data):
        crc_table = [0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401, 0xA001, 0x6C00, 0x7800, 0xB401, 0x5000,
                    0x9C01, 0x8801, 0x4400]

        len = 22
        crcTmp = 0xFFFF
        while (len > 0):
            len = len - 1
            tmp = crc_table[(data[22 - len - 1] ^ crcTmp) & 15] ^ (crcTmp >> 4)
            crcTmp = crc_table[((data[22 - len - 1] >> 4) ^ tmp) & 15] ^ (tmp >> 4)

        return crcTmp

# ------------------------------------------------主要執行序-------------------------------------------------------------------
    
# 定義 Class
msg = ReceiveMsg()
cmd = Command()
crc = CrcTmp()

# 定義 連線
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)      # Create TCP socket
s.connect((DEVICE_IP, DEVICE_PORT))                        # Connect to device

def main():
    try:

        # 在程式啟動時，只發布一次雷射測距控制命令 (此處 1 表示開啟)**
        laser_cmd_bytes = Command.laser_command(1)
        s.send(laser_cmd_bytes)
        print("雷射測距命令已發布 (開啟雷射)")

        # 循環發布'回傳'命令 - 
        while True:
            s.send(bytearray(rec_cmd))    # **送出命令**
            buffer = s.recv(buffer_size)  # **接收回應資料**

            if msg.parse(buffer, len(buffer), msg):
                print(f"roll:{msg.rollAngle}, yaw:{msg.yawAngle}, pitch:{msg.pitchAngle}")
                print(f"測距: {msg.targetDist}")
            else:
                print("Can't Received The Data")

            # 10Hz 傳送命令
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("使用者中斷程式執行")
    except Exception as e:
        print("發生例外:", e)
    finally:
        s.close()  # **確保在任何情況下都會關閉 socket**

if __name__ == "__main__":
    main()