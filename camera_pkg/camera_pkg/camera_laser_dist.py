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

# 地球半徑 單位:M(公尺)
Earth_radius = 6371000.0

# ** 定義一個函數，利用起始經緯度、方位角與水平距離，計算目的地經緯度 **
def destination_point(lat, lon, bearing, distance):
    
    R = Earth_radius
    # 將起始經緯度與方位角轉換成弧度
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing)

    # 計算目的地的緯度 (公式來源：球面三角學)
    lat2 = math.asin(math.sin(lat_rad) * math.cos(distance / R) +
                     math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad))
    # 計算目的地的經度
    lon2 = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
                                math.cos(distance / R) - math.sin(lat_rad) * math.sin(lat2))
    # 將結果轉換回角度並回傳
    return math.degrees(lat2), math.degrees(lon2)

# 定義 ROS2 節點類別，處理雲台資料與 UAV 的 GPS 與 Heading 資料
class TargetPositionNode(Node):
    def __init__(self):
        super().__init__('target_position_node')

        qos=QoSProfile(
			reliability=ReliabilityPolicy(0),
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )
        
        # 訂閱雲台資料(camera_data_pub)
        self.create_subscription(
            Camera,
            '/camera_data_pub',
            self.camera_callback,
            qos_profile=qos)
        
        # 訂閱 UAV 的 GPS 資料 (NavSatFix)，包含經緯度與海拔高度
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_callback,
            qos_profile=qos)
        
        # 訂閱 UAV 的方位角資料 (Heading)
        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_callback,
            qos_profile=qos)
        
        # 用來存放最新接收到的 UAV GPS 與 Heading 資料
        self.uav_lat = None      # 飛機當前緯度
        self.uav_lon = None      # 飛機當前經度
        self.uav_alt = None      # 飛機高度（海拔，單位：公尺）
        self.uav_heading = None  # 飛機方位角（單位：度）
    
    # GPS 訊息的 callback 函數
    def gps_callback(self, msg: NavSatFix):
        # 儲存最新的經緯度與海拔高度
        self.uav_lat = msg.latitude   # 飛機緯度
        self.uav_lon = msg.longitude  # 飛機經度
        self.uav_alt = msg.altitude   # 飛機高度（海拔)
        self.get_logger().info(f'[GPS] lat: {self.uav_lat}, lon: {self.uav_lon}, alt: {self.uav_alt}')
    
    # ** Heading 訊息的 callback 函數 **
    def heading_callback(self, msg: Float64):
        # ** 儲存最新的飛機方位角 **
        self.uav_heading = msg.data  # ** 飛機方位角（單位：度） **
        self.get_logger().info(f'[Heading] {self.uav_heading} 度')
    
    # ** 雲台資料的 callback 函數 **
    def camera_callback(self, msg: Camera):
        # ** 檢查雲台資料陣列是否有數據 **
        if not msg.data:
            self.get_logger().warning('沒有接收到雲台數據！')
            return
        
        # ** 這裡取第一筆資料進行計算 **
        camera_data = msg.data[0]
        # ** 從訊息中取得雷射測距距離（目標物距離）、pitch 與 yaw 角度 **
        target_distance = camera_data.targetdist  # ** 雷射測距的距離（單位：公尺） **
        pitch_angle = camera_data.pitchangle      # ** 雲台的 pitch 角度（單位：度） **
        gimbal_yaw = camera_data.yawangle         # ** 雲台的 yaw 角度（單位：度） **
        
        self.get_logger().info(f'[Camera] target_distance: {target_distance}, pitch: {pitch_angle}, yaw: {gimbal_yaw}')
        
        # ** 檢查 UAV 的 GPS 與 Heading 資料是否都有接收到 **
        if self.uav_lat is None or self.uav_lon is None or self.uav_heading is None:
            self.get_logger().warning('等待 UAV GPS 或 Heading 資料...')
            return
        
        # ** 將角度從度轉換成弧度以進行三角函數計算 **
        pitch_rad = math.radians(pitch_angle)
        
        # ** 計算水平距離：利用雷射測距距離乘上 cos(pitch) 得到水平分量 **
        horizontal_distance = target_distance * math.cos(pitch_rad)
        # ** 注意：如果雷射的量測方向與地面交會處並非正好對應，則可能需要結合 UAV 的高度修正。此處假設雷射是直接測量到目標距離 **
        
        # ** 計算目標的相對方位角：將 UAV 的方位角與雲台 yaw 角疊加 **
        # ** 假設雲台 yaw 為相對於 UAV 前方的偏角，最終轉成相對於正北的絕對方位角 **
        total_bearing = (self.uav_heading + gimbal_yaw) % 360
        self.get_logger().info(f'[計算] 水平距離: {horizontal_distance:.2f} 公尺, 絕對方位角: {total_bearing:.2f} 度')
        
        # ** 使用起始點 (UAV 的 GPS 經緯度) 與計算得到的水平距離、方位角，利用球面坐標公式計算目標的經緯度 **
        target_lat, target_lon = destination_point(self.uav_lat, self.uav_lon, total_bearing, horizontal_distance)
        self.get_logger().info(f'[結果] 目標經度: {target_lon:.6f}, 目標緯度: {target_lat:.6f}')

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
