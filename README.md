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

  - Example

    ```
    ros2 run camera_tt30_pkg camera_feedback_publisher_gui_node
    ```

    ---

</details>

---
