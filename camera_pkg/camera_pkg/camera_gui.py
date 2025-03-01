#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File   : camera_command.py
Author : LYX(先驅), FantasyWilly
Email  : FantasyWilly - bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 : 創建簡單 GUI 界面，用於發送相機控制指令
"""

import tkinter as tk
import camera_command as cm  # 請確認此模組路徑正確

class CameraControlGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Directional Buttons")
        self.create_widgets()

    def create_widgets(self):

        # ----------------- 群組1: Common -----------------
        group1_frame = tk.Frame(self.master)
        group1_frame.pack(padx=10, pady=5, fill='x')
        group1_label = tk.Label(group1_frame, text="Common", font=("Helvetica", 12, "bold"),
                                 anchor="center", justify="center")
        group1_label.pack(side="top", fill="x", padx=5)
        group1_buttons = tk.Frame(group1_frame)
        group1_buttons.pack(fill='x', padx=5)
        btn_netural = tk.Button(group1_buttons, text="一鍵回中", command=self.on_netural)
        btn_netural.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_down = tk.Button(group1_buttons, text="一鍵向下", command=self.on_down)
        btn_down.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_follow = tk.Button(group1_buttons, text="跟隨機頭", command=self.on_follow_header)
        btn_follow.pack(side='left', padx=5, pady=5, expand=True, fill='x')

        # ----------------- 群組2: Photo -----------------
        group2_frame = tk.Frame(self.master)
        group2_frame.pack(padx=10, pady=5, fill='x')
        group2_label = tk.Label(
            group2_frame,
            text="Photo",
            font=("Helvetica", 12, "bold"),
            anchor="center",
            justify="center"
        )
        group2_label.pack(side="top", fill="x", padx=5)
        group2_buttons = tk.Frame(group2_frame)
        group2_buttons.pack(fill='x', padx=5)
        btn_take_photo = tk.Button(group2_buttons, text="拍一張照片", command=self.on_take_photo)
        btn_cont_shoot = tk.Button(group2_buttons, text="拍三張照片", command=self.on_continuous_shooting)
        btn_take_photo.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_cont_shoot.pack(side='left', padx=5, pady=5, expand=True, fill='x')

        # ----------------- 群組3: Video -----------------
        group3_frame = tk.Frame(self.master)
        group3_frame.pack(padx=10, pady=5, fill='x')
        group3_label = tk.Label(group2_frame, text="Video", font=("Helvetica", 12, "bold"),
                                 anchor="center", justify="center")
        group3_label.pack(side="top", fill="x", padx=5)
        group3_buttons = tk.Frame(group2_frame)
        group3_buttons.pack(fill='x', padx=5)
        btn_start = tk.Button(group3_buttons, text="開始錄影", command=self.on_start_recording)
        btn_stop = tk.Button(group3_buttons, text="結束錄影", command=self.on_stop_recording)
        btn_start.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        btn_stop.grid(row=0, column=1, sticky="ew", padx=5, pady=5)
        group3_buttons.columnconfigure(0, weight=1)
        group3_buttons.columnconfigure(1, weight=1)

        # ----------------- 群組4: Zoom -----------------
        group4_frame = tk.Frame(self.master)
        group4_frame.pack(padx=10, pady=5, fill='x')
        group4_label = tk.Label(group4_frame, text="Zoom", font=("Helvetica", 12, "bold"),
                                 anchor="center", justify="center")
        group4_label.pack(side="top", fill="x", padx=5)
        group4_buttons = tk.Frame(group4_frame)
        group4_buttons.pack(fill='x', padx=5)
        btn_reset = tk.Button(group4_buttons, text="恢復1倍", command=self.on_zoom_reset)
        btn_zoom_in = tk.Button(group4_buttons, text="放大2倍", command=self.on_zoom_in)
        btn_zoom_out = tk.Button(group4_buttons, text="縮小2倍", command=self.on_zoom_out)
        btn_reset.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_zoom_in.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_zoom_out.pack(side='left', padx=5, pady=5, expand=True, fill='x')

        # ----------------- 群組5: Laser -----------------
        group5_frame = tk.Frame(self.master)
        group5_frame.pack(padx=10, pady=5, fill='x')
        group5_label = tk.Label(
            group5_frame,
            text="Laser",
            font=("Helvetica", 12, "bold"),
            anchor="center",
            justify="center"
        )
        group5_label.pack(side="top", fill="x", padx=5)
        group5_buttons = tk.Frame(group5_frame)
        group5_buttons.pack(fill='x', padx=5)
        btn_laser_open = tk.Button(group5_buttons, text="打開雷射", command=self.on_laser_open)
        btn_laser_close = tk.Button(group5_buttons, text="關閉雷射", command=self.on_laser_close)
        btn_laser_open.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_laser_close.pack(side='left', padx=5, pady=5, expand=True, fill='x')

        # ----------------- 群組6: Direction (上下左右) -----------------
        group6_frame = tk.Frame(self.master)
        group6_frame.pack(padx=10, pady=5, fill='x')
        group6_label = tk.Label(group6_frame, text="Direction", font=("Helvetica", 12, "bold"),
                                anchor="center", justify="center")
        group6_label.pack(side="top", fill="x", padx=5)

        group6_buttons = tk.Frame(group6_frame)
        group6_buttons.pack(padx=5, pady=5)


        btn_up = tk.Button(group6_buttons, text="▲", width=3, height=2, command=self.on_indicator_up)
        btn_up.grid(row=0, column=1, padx=5, pady=5)

        btn_left = tk.Button(group6_buttons, text="◀", width=3, height=2, command=self.on_indicator_left)
        btn_left.grid(row=1, column=0, padx=5, pady=5)

        btn_right = tk.Button(group6_buttons, text="▶", width=3, height=2, command=self.on_indicator_right)
        btn_right.grid(row=1, column=2, padx=5, pady=5)

        btn_down = tk.Button(group6_buttons, text="▼", width=3, height=2, command=self.on_indicator_down)
        btn_down.grid(row=2, column=1, padx=5, pady=5)

    # ----------------- Common -------------------
    def on_netural(self):
        cm.Command.Netural_command()
        print("指令: 一鍵回中 pressed")

    def on_down(self):
        cm.Command.Down_command()
        print("指令: 一鍵向下 pressed")

    def on_follow_header(self):
        cm.Command.FollowHeader_command()
        print("指令: 跟隨機頭 pressed")

    # ----------------- Photo -------------------
    def on_take_photo(self):
        cm.Command.Photo_command(1, 0)
        print("指令: 拍一張照片 pressed")
    
    def on_continuous_shooting(self):
        cm.Command.Photo_command(2, 3)
        print("指令: 拍三張照片 pressed")

    # ----------------- Video -------------------
    def on_start_recording(self):
        cm.Command.Video_command(1)
        print("指令: 開始錄影 pressed")

    def on_stop_recording(self):
        cm.Command.Video_command(2)
        print("指令: 結束錄影 pressed")

    # ------------------ Zoom -------------------
    def on_zoom_reset(self):
        cm.Command.MachineZoom_command(4)
        print("指令: 恢復1倍 pressed")

    def on_zoom_in(self):
        cm.Command.MachineZoom_command(5)
        print("指令: 放大2倍 pressed")

    def on_zoom_out(self):
        cm.Command.MachineZoom_command(6)
        print("指令: 縮小2倍 pressed")

    # ------------------ Laser ------------------
    def on_laser_open(self):
        cm.Command.Laser_command(1)
        print("指令: 打開雷射 pressed")

    def on_laser_close(self):
        cm.Command.Laser_command(2)
        print("指令: 關閉雷射 pressed")

    # ----------------- Direction -----------------
    def on_indicator_up(self):
        cm.Command.Indicator_command(0, 500)
        print("指令: 向上 pressed")

    def on_indicator_down(self):
        cm.Command.Indicator_command(0, -500)
        print("指令: 向下 pressed")

    def on_indicator_left(self):
        cm.Command.Indicator_command(-500, 0)
        print("指令: 向左 pressed")

    def on_indicator_right(self):
        cm.Command.Indicator_command(500, 0)
        print("指令: 向右 pressed")

# ------------------ 主要執行序 -------------------
def main():
    root = tk.Tk()
    app = CameraControlGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
