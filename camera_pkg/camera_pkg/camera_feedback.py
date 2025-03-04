#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
File   : camera_feedback.py
author : LYX(先驅), FantasyWilly
email  : FantasyWilly - bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 : 接收雲台資訊並發布至ROS2
    1. 接收 - 相機＆雲台回傳
    2. 打開 - 雷射測距
    3. 持續傳送回傳指令以取得數據
'''

import socket
import time
import threading

import camera_pkg.camera_decoder as decoder

# 定義設備 IP 與 Port (從 communication 模組中取得)
DEVICE_IP = "192.168.144.200"
DEVICE_PORT = 2000

# 定義回傳命令與緩衝區大小
rec_cmd = [0x4B, 0x4B, 0x01, 0x97]    # 回傳 'Data' 命令控制，注意：這裡是持續發送的指令
buffer_size = 32                      # 接收 'Data' 資料長度

# 全域建立一個 ReceiveMsg 物件，用以儲存解析後的數據
msg = decoder.ReceiveMsg()

# 建立 socket 並連線至設備
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 建立 TCP socket
s.connect((DEVICE_IP, DEVICE_PORT))                    # 連線到設備

# 定義一個函數，用來持續傳送回傳指令
def send_rec_cmd():
    while True:
        try:
            s.send(bytes(rec_cmd))
        except Exception as e:
            print("傳送指令時發生例外:", e)
            break
        time.sleep(0.1)

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

        # 建立傳送指令的執行緒，讓其持續傳送回傳指令
        sender_thread = threading.Thread(target=send_rec_cmd, daemon=True)
        sender_thread.start()

        # 循環讀取並解析封包，從 communication 模組中取得完整封包
        for packet in decoder.ReceiveMsg.recv_packets(s, packet_size=32, header=b'\x4B\x4B'):
            if msg.parse(packet, len(packet), msg):
                print(f"roll: {msg.rollAngle}, yaw: {msg.yawAngle}, pitch: {msg.pitchAngle}")
                print(f"測距: {msg.targetDist}")
            else:
                print("Can't Received The Data")
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("使用者中斷程式執行")
    except Exception as e:
        print("發生例外:", e)
    finally:
        s.close()

if __name__ == "__main__":
    main()
