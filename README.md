# <div align="center">AESIL - Camera</div>

## <div align="center">DOWNLOADS</div>

__*Step1* 下載至 ROS2 的工作空間__
```bash
cd ~/<your_workspace>/src
git clone https://github.com/FantasyWilly/AESIL_Camera.git
```
---

__*Step2* 編譯工作空間__
```bash
colon build
source ~/.bashrc
```
---

## <div align="center">Camera</div>

  ### [ D-80 系列 ] - (Pro)

  - **ROS2 Running**

    ```bash
    ros2 run camera_d80_pkg guc_ro2_main_node
    ```

    `Node:` gcu_ros2_main_node
  
    `Topic:` camera_data_pub
    
    ---

  ### [ KTG 系列 ] - (TT30)

  - **ROS2 Running - (Optional)**

    ```bash
    ros2 run camera_tt30_pkg camera_feedback_publisher_gui_node
    ```

    `Node:` gcu_ros2_main_node

    `Topic:` /camera_data_pub, /laser_data_pub

    ---

  - **ROS2 Launch - (Optional)**

    ```bash
    ros2 launch camera_tt30_pkg camera_gui_ros2_launch.py
    ```

  - **Config**

    ```yaml
    camera_feedback_publisher_gui_node:
    ros__parameters:
        gimbal_step: 50                   # 雲台移動度數 (gimbal_step/10)
        zoom_duration: 0.3                # 持續放大縮小時間 (s)
        photo_continous_count: 3          # 連續拍照次數
    ```
    
    ---

  - **[ A. 天空端 (Linux) ] ROS2 Xbox - (Optional)**

    ```bash
    ros2 run camera_tt30_pkg xbox_air_node
    ```

  - **[ B. 地面端 (Linux) ] Xbox**

    ```bash
    python3 camera_tt30_pkg/camera_tt30_pkg/xbox_ground.py
    ```

  - **[ B. 地面端 (Windows) ] Xbox**

    ```bash
    python3 camera_tt30_pkg/camera_tt30_pkg/windows_ground/xbox_ground.py
    ```

    ---

## <div align="center">YOLO</div>