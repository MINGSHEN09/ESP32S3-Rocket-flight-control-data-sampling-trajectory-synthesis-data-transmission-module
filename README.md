# ESP32S3-Rocket flight control, data sampling, trajectory synthesis, data transmission module
This project based on ESP32S3,ICM-20602,NEO-M8N,BMP388 and SIM7600CET
# 🚀 ESP32-S3 火箭遥测系统

基于 ESP32-S3 的火箭飞行数据采集与回传项目  
采集 **位置、轨迹、海拔、姿态**，通过 **4G 实时回传** + **TF卡黑匣子存储**

![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

## 项目目标

- 实时采集火箭飞行数据（加速度、角速度、GPS 位置、高度、气压）
- 使用 SIM7600CE-T 通过 4G 把关键数据发回地面站（或云端）
- 所有原始数据保存到 TF 卡（即使 4G 断开也能保留完整记录）
- 适合模型火箭、高空实验或业余火箭测试

## 硬件清单

| 部件          | 型号          | 作用                           | 通讯方式 | 推荐引脚（可修改）          |
|---------------|---------------|--------------------------------|----------|-----------------------------|
| 主控          | ESP32-S3      | 核心处理、任务调度             | —        | —                           |
| IMU           | ICM-20602     | 加速度 + 角速度（姿态/振动）   | SPI      | CS=10, SCK=12, MOSI=11, MISO=13 |
| GPS           | NEO-M8N       | 经纬度、速度、高度、时间       | UART     | TX=17, RX=18                |
| 气压高度计    | BMP388        | 高精度相对高度（抗 GPS 漂移）  | I²C      | SDA=21, SCL=22              |
| 4G 模块       | SIM7600CE-T   | LTE 数据回传                   | UART     | TX=33, RX=32                |
| 存储          | MicroSD 卡    | 数据黑匣子（CSV 格式）         | SPI      | CS=5                        |
| 电源          | 3.7–5V LiPo   | 供电（SIM7600 需大电流）       | —        | 加 ≥1000μF 电容稳压         |

**重要提醒**：SIM7600 峰值电流可达 2A 以上，建议单独 5V 供电 + 大电容，否则容易重启或死机。

📌### 📌 硬件连线 (Wiring Map)

#### 1. I2C 传感器 (共享总线)
| 传感器信号 (VCC=3.3V) | ESP32-S3 引脚 |
| :--- | :--- |
| **SCL** | GPIO 22 |
| **SDA** | GPIO 21 |

#### 2. SPI SD 卡模块
| SD 模块引脚 | ESP32-S3 引脚 |
| :--- | :--- |
| **CS** | GPIO 5 |
| **MOSI** | GPIO 11 |
| **MISO** | GPIO 13 |
| **SCK** | GPIO 12 |

#### 3. UART 串口设备
| 设备 | 信号 | ESP32-S3 引脚 |
| :--- | :--- | :--- |
| **NEO-M8N** | TX / RX | GPIO 18 / 19 |
| **SIM7600** | TX / RX | GPIO 17 / 16 |
| **SIM7600** | PWRKEY | GPIO 4 |

📦 库依赖项
需在Arduino IDE 中安装以下库：
 * Adafruit_BMP3XX
 * TinyGPSPlus
 * TinyGSM
 * ArduinoHttpClient
 * ICM20602 (或对应的驱动代码)
☁️ 服务器端部署
项目包含一个简单的 Python Flask 接收端，部署在阿里云服务器上：
 * 安装环境：
   pip3 install flask
 * 启动服务：
   运行 server.py，确保服务器安全组已开放 5000 端口。
 * 数据格式：
   服务器将接收如下格式的 JSON：
   {"ph": 1, "lat": 39.9, "lon": 116.3, "alt": 120.5, "v": 15.2}

⚠️ 安全警告与免责声明
 * 法律合规：请在进行任何火箭发射活动前，确保符合当地航空管理部门的法律法规。
 * 电源安全：SIM7600 模块在发射信号时会有大电流波动，请务必使用独立电源或加装大电容。
 * 免责声明：本项目仅供科研和学习参考，因使用本代码导致的任何硬件损坏或意外事故，作者概不负责。
