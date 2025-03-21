# <div align="center">AESIL - Camera</div>

## <div align="center">Downloads</div>

### *Step1* 下載至 ROS2 的工作空間

```
cd ~/<your_workspace>/src
git clone https://github.com/FantasyWilly/AESIL_Camera.git
```

---
### *Step2* 編譯工作空間
```
colon build
source ~/.bashrc
```
---

## <div align="center">Introduce & Use</div>

<details>
  <summary>D-80 系列</summary>

  ### <div align="center">D-80 Pro</div>

  - Example

    ```
    ros2 run camera_d80_pkg guc_ro2_main_node
    ```

    ---

</details>

<details>
  <summary>KTG 系列</summary>

  ### <div align="center">KTG-TT30</div>

  **ROS2 Running**

  ```bash
  ros2 run camera_tt30_pkg camera_feedback_publisher_gui_node
  ```

  ---

  **ROS2 Launch**

  ```bash
  ros2 launch camera_tt30_pkg camera_gui_ros2_launch.py
  ```

  **Config**

  ```yaml
  camera_feedback_publisher_gui_node:
  ros__parameters:
      gimbal_step: 50                   # 雲台移動度數 (gimbal_step/10)
      zoom_duration: 0.3                # 持續放大縮小時間 (s)
      photo_continous_count: 3          # 連續拍照次數
  ```

</details>

---
