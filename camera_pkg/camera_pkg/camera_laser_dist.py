#!/usr/bin/env python3

'''
File   : camera_laser_dist.py
author : FantasyWilly
email  : bc697522h04@gmail.com
'''

'''
相機型號 : KTG-TT30
檔案大綱 :
    1. 接收 - 相機＆雲台回傳資料
    2. 計算 - 目標物經緯度
    3. 發布 - 目標物經緯度資訊至ROS2
'''

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from camera_msg_pkg.msg import Camera
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

# ----------------------- 基本參數設定 -----------------------

# ** 定義地球半徑（單位：公尺） **
Earth_radius = 6371000.0

# ** 定義一個函數，利用起始經緯度、方位角與水平距離計算目的地經緯度 **
def destination_point(lat, lon, bearing, distance):
    R = Earth_radius
    # ** 將起始經緯度與方位角轉換成弧度以便進行計算 **
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing)

    # ** 計算目的地的緯度（使用球面三角學公式） **
    lat2 = math.asin(math.sin(lat_rad) * math.cos(distance / R) +
                     math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad))
    # ** 計算目的地的經度（使用球面三角學公式） **
    lon2 = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
                                math.cos(distance / R) - math.sin(lat_rad) * math.sin(lat2))
    # ** 將計算結果從弧度轉換回度並回傳 **
    return math.degrees(lat2), math.degrees(lon2)

# ** 定義 ROS2 節點類別，用於處理雲台資料、UAV 的 GPS、Heading 與相對高度資料 **
class TargetPositionNode(Node):
    def __init__(self):
        # ** 初始化節點名稱為 'target_position_node' **
        super().__init__('target_position_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy(0),          # ** 使用 BEST EFFORT 模式 **
            durability=DurabilityPolicy.VOLATILE,          # ** 設定耐久性為 VOLATILE **
            history=HistoryPolicy.KEEP_ALL,                # ** 保留所有訊息 **
        )
        
        # ** 訂閱雲台資料 (Camera) ，話題名稱：/camera_data_pub **
        self.create_subscription(
            Camera,
            '/camera_data_pub',
            self.camera_callback,
            qos_profile=qos)
        
        # ** 訂閱 UAV 的 GPS 資料 (NavSatFix) ，只取得經緯度資訊 **
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_callback,
            qos_profile=qos)
        
        # ** 訂閱 UAV 的方位角資料 (Heading) ，話題名稱：/mavros/global_position/compass_hdg **
        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_callback,
            qos_profile=qos)
        
        # ** 訂閱 UAV 的相對高度資料，話題名稱：/mavros/global_position/rel_alt **
        self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.rel_alt_callback,
            qos_profile=qos)
        
        # ** 初始化變數，用以儲存接收到的最新資料 **
        self.uav_lat = None         # ** UAV 的緯度 **
        self.uav_lon = None         # ** UAV 的經度 **
        self.uav_rel_alt = None     # ** UAV 的相對高度（公尺） **
        self.uav_heading = None     # ** UAV 的方位角（度） **
        
        # ** 建立一個 Publisher 用來發布目標的經緯度資訊，使用 NavSatFix 訊息，話題名稱為 /target_position **
        self.target_pub = self.create_publisher(
            NavSatFix,
            '/target_position',
            qos_profile=qos)

    # ** GPS 訊息的 callback 函數：接收並儲存 UAV 的經緯度 **
    def gps_callback(self, msg: NavSatFix):
        self.uav_lat = msg.latitude    # ** 儲存 UAV 緯度 **
        self.uav_lon = msg.longitude   # ** 儲存 UAV 經度 **
        self.get_logger().info(f'[GPS] lat: {self.uav_lat}, lon: {self.uav_lon}')
    
    # ** 相對高度的 callback 函數：接收並儲存 UAV 的相對高度 **
    def rel_alt_callback(self, msg: Float64):
        self.uav_rel_alt = msg.data   # ** 儲存 UAV 的相對高度（公尺） **
        self.get_logger().info(f'[相對高度] {abs(self.uav_rel_alt):.2f} 公尺')
    
    # ** Heading 訊息的 callback 函數：接收並儲存 UAV 的方位角 **
    def heading_callback(self, msg: Float64):
        self.uav_heading = msg.data   # ** 儲存 UAV 方位角（度） **
        self.get_logger().info(f'[Heading] {self.uav_heading} 度')
    
    # ** 雲台資料的 callback 函數：根據接收到的數據計算目標物經緯度 **
    def camera_callback(self, msg: Camera):
        # ** 檢查是否有接收到雲台數據，若無則返回 **
        if not msg.data:
            self.get_logger().warning('沒有接收到雲台數據！')
            return
        
        # ** 從雲台數據中取出第一筆進行計算 **
        camera_data = msg.data[0]
        # ** 提取雷射測距距離、pitch 與 yaw 角度 **
        target_distance = camera_data.targetdist  # ** 雷射測距距離（公尺） **
        pitch_angle = camera_data.pitchangle       # ** 雲台的 pitch 角度（度） **
        gimbal_yaw = camera_data.yawangle          # ** 雲台的 yaw 角度（度） **
        
        self.get_logger().info(f'[Camera] target_distance: {target_distance}, pitch: {pitch_angle}, yaw: {gimbal_yaw}')
        
        # ** 檢查是否已接收到 UAV 的經緯度、方位角與相對高度資料 **
        if self.uav_lat is None or self.uav_lon is None or self.uav_heading is None or self.uav_rel_alt is None:
            self.get_logger().warning('等待 UAV GPS、Heading 或相對高度資料...')
            return
        
        # ** 將 pitch 角度由度轉換成弧度以便進行三角計算 **
        pitch_rad = math.radians(pitch_angle)
        
        # ** 計算水平距離：利用雷射測距距離乘以 cos(pitch) 得到水平方向分量 **
        horizontal_distance = target_distance * math.cos(pitch_rad)
        
        # ** 計算目標的絕對方位角：將 UAV 的方位角與雲台 yaw 相加，再取模 360 得到 0~359 度之間 **
        # ** 將雲台 yaw 轉換為 0～360 度範圍 **
        gimbal_yaw_adjusted = (gimbal_yaw + 360) % 360  # ** 若 gimbal_yaw 為負值，加上 360 可取得相對應的正值，確保數值在 0 到 359 度之間 **

        # ** 計算目標的絕對方位角：將 UAV 的方位角與調整後的雲台 yaw 相加，再取模 360 **
        total_bearing = (self.uav_heading + gimbal_yaw_adjusted) % 360  # ** 結合 UAV 航向與雲台 yaw 後，使用模運算確保結果介於 0~359 度之間 **

        self.get_logger().info(f'[計算] 水平距離: {horizontal_distance:.2f} 公尺, 絕對方位角: {total_bearing:.2f} 度')

        
        # ** 利用 UAV 的經緯度、水平距離與方位角，透過球面坐標公式計算目標物的經緯度 **
        target_lat, target_lon = destination_point(self.uav_lat, self.uav_lon, total_bearing, horizontal_distance)
        self.get_logger().info(f'[結果] 目標緯度: {target_lat:.7f}, 目標經度: {target_lon:.7f}')
        self.get_logger().info(f'[結果] UAV 相對高度: {abs(self.uav_rel_alt):.1f} 公尺')
        
        # ** 建立並填充 NavSatFix 訊息，用於發布目標的經緯度資訊 **
        target_msg = NavSatFix()
        target_msg.latitude = round(target_lat, 7)    # ** 設定目標緯度 **
        target_msg.longitude = round(target_lon, 7)   # ** 設定目標經度 **
        target_msg.altitude = self.uav_rel_alt  # ** 可選：設定目標高度（此處以 UAV 相對高度為例） **
        
        # ** 將目標位置訊息發布到 /target_position 話題 **
        self.target_pub.publish(target_msg)
        # self.get_logger().info(f'[發布] 已發布目標位置至 /target_position')
    
def main(args=None):
    rclpy.init(args=args)
    node = TargetPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # ** 清理並關閉節點 **
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
