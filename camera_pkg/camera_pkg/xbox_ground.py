#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
檔案   : ground_controller.py
作者   : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 : 
    A. 地面端透過 Xbox 控制器發送控制命令
"""

# Python
import time
import sys
import pygame

# ROS2
import camera_pkg.camera_command as cm
from camera_pkg.camera_communication import CommunicationController


# ---------- 基本參數 (全域) ----------
# 如果你是直連相機，請使用相機 IP 與埠號；如果是透過代理服務，請改為代理服務的 IP 與埠號
DEVICE_IP = "192.168.0.230"     # 或改成代理服務所在的 IP
DEVICE_PORT = 9999              # 或代理服務的埠號

# 每次雲台調整的角度增量（單位：度）
CONTROL_INCREMENT = 5.0

def xbox_controller_loop(controller: CommunicationController) -> None:
    """
    使用 pygame 監聽 Xbox 控制器事件，
    依據控制器操作呼叫對應的命令函式，
    當按下按鈕 11 時結束程式。
    (這個版本是單線程的，所有事件都在主線程中處理)
    """
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("未找到 Xbox 控制器")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Xbox 控制器已啟動")

    # 進入事件循環
    while True:
        for event in pygame.event.get():
            # 處理按鈕按下事件
            if event.type == pygame.JOYBUTTONDOWN:
                # 若按下按鈕 11，結束程式
                if event.button == 7:
                    print("按下按鈕 7, 程式將結束")
                    pygame.quit()
                    return  # 直接返回結束事件循環

                # 依據其他按鈕發送命令
                if joystick.get_button(0):
                    print("A 按鈕按下：向下")
                    cm.Command.Down_command(controller)
                elif joystick.get_button(1):
                    print("B 按鈕按下：拍照")
                    cm.Command.Photo_command(controller)
                elif joystick.get_button(2):
                    print("X 按鈕按下：跟隨機頭")
                    cm.Command.FollowHeader_command(controller)
                elif joystick.get_button(3):
                    print("Y 按鈕按下：回中")
                    cm.Command.Netural_command(controller)
                elif joystick.get_button(4):
                    print("L 按鈕按下：開始錄影")
                    cm.Command.Video_command(controller, 1)
                elif joystick.get_button(5):
                    print("R 按鈕按下：結束錄影")
                    cm.Command.Video_command(controller, 2)

            # 處理方向鍵 (hat) 事件，用以控制雲台角度
            elif event.type == pygame.JOYHATMOTION:
                hat = joystick.get_hat(0)
                if hat != (0, 0):
                    pitch = hat[1] * CONTROL_INCREMENT
                    yaw   = hat[0] * CONTROL_INCREMENT
                    print(f"Hat 更新：發送雲台控制指令 -> pitch: {pitch}°, yaw: {yaw}°")
                    cm.Command.GimbalControl_command(controller, pitch=pitch * 10, yaw=yaw * 10)
            
            # 處理觸發器 (axis) 事件，用以控制 zoom
            elif event.type == pygame.JOYAXISMOTION:
                # 假設 RT 為軸 5；當值大於 0.5 表示按下，否則釋放
                if event.axis == 5:
                    rt_value = joystick.get_axis(5)
                    if rt_value > 0.5:
                        print("RT 按下：放大")
                        cm.Command.MachineZoom_command(controller, 1)
                    else:
                        print("RT 釋放：停止放大縮小")
                        cm.Command.MachineZoom_command(controller, 3)
                # 假設 LT 為軸 2；當值大於 0.5 表示按下，否則釋放
                elif event.axis == 2:
                    lt_value = joystick.get_axis(2)
                    if lt_value > 0.5:
                        print("LT 按下：縮小")
                        cm.Command.MachineZoom_command(controller, 2)
                    else:
                        print("LT 釋放：停止放大縮小")
                        cm.Command.MachineZoom_command(controller, 3)
        time.sleep(0.1)

# ----------------------- [main] 主要執行序 -----------------------
def main() -> None:
    controller = CommunicationController(DEVICE_IP, DEVICE_PORT)
    try:
        controller.connect()
        print("[連線] 嵌入式第腦")
        
        # 直接在主線程中進入事件循環
        xbox_controller_loop(controller)
        
    except Exception as e:
        print("[main] 出現錯誤:", e)
    finally:
        controller.disconnect()
        print("連線已關閉")

if __name__ == "__main__":
    main()
