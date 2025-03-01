#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''

File   : camera_command.py

author : LYX(先驅), FantasyWilly
email  : bc697522h04@gmail.com

'''

'''

相機型號 : KTG-TT30
檔案大綱 : 控制命令
    1. 固定格式
    2. 命令程式控制

'''

# ------------------------------------- 固定程式 --------------------------------------------- 

# FIXED_BYTES : KTG-TT30 的雲台控制前面固定指令
FIXED_BYTES = (b'\x4B\x4B\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x88')

# 定義光源控制位元：可見光與熱成像
VISIBLE_LIGHT = b'\x01'   # 可見光
THERMAL_IMAGING = b'\x02' # 熱成像

# 補齊字元    
class Pad:
    def pad_to_8_bytes(bytes_check):

        assert isinstance(bytes_check,(bytes,bytearray)) and len (bytes_check) <= 23

        padding_needed = 23 -len(bytes_check)

        padding = bytes_check + b'\x00' * padding_needed

        return padding

# 校驗碼
class CrcTmp:
    def calc(data: bytes) -> int:
        crc_tmp = 0
        for b in data:
            crc_tmp += b
        return crc_tmp

# ------------------------------------- 命令控制 ---------------------------------------------

# 命令控制
class Command:

    # 回中
    def Netural_command():
        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT
        
        send_bytes = send_bytes + (b'\x03')  
        send_bytes = Pad.pad_to_8_bytes(send_bytes)

        crc = CrcTmp.calc(send_bytes)
        send_bytes = send_bytes + crc.to_bytes(2, 'little')       

        return send_bytes
    
    # 向下
    def Down_command():
        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT
        
        send_bytes = send_bytes + (b'\x07')  
        send_bytes = Pad.pad_to_8_bytes(send_bytes)

        crc = CrcTmp.calc(send_bytes)
        send_bytes = send_bytes + crc.to_bytes(2, 'little')       

        return send_bytes

    # 雷射測距開關 : opecn_or_close = (0)關閉, (1)開啟
    def laser_command(open_or_close):

        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT
        
        send_bytes = send_bytes + (b'\x21')
        send_bytes = send_bytes + open_or_close.to_bytes(2, 'little')  
        send_bytes = Pad.pad_to_8_bytes(send_bytes)

        crc = CrcTmp.calc(send_bytes)
        send_bytes = send_bytes + crc.to_bytes(2, 'little')       

        return send_bytes
    
if __name__ == "__main__":
    Command.Netural_command()
    Command.Down_command()
    Command.laser_command(0)
    