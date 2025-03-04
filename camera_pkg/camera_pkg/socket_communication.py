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

# 定義相機連線參數
TCP_IP = "192.168.144.200"       # 相機 IP
TCP_PORT = 2000                  # 相機 Port
BUFFER_SIZE = 32                 # 接收封包位元組長度

def send_command(cmd_bytes):
    """
    發送命令到相機
    :param cmd_bytes: 位元組格式的命令
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((TCP_IP, TCP_PORT))
        s.send(cmd_bytes)
        print("[發送]: 成功")
        print("-----------------------------")
    except socket.error as e:
        print("[發送]: !!失敗!!")
        print("[錯誤訊息]:", e)
        print("-----------------------------")
    finally:
        # 關閉連線
        s.close()
