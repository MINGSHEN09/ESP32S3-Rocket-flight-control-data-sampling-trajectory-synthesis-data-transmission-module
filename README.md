# ESP32S3-Rocket flight control, data sampling, trajectory synthesis, data transmission module
This project based on ESP32S3,ICM-20602,NEO-M8N,BMP388 and SIM7600CET
# 🚀 ESP32-S3 火箭遥测系统

基于 ESP32-S3 的火箭飞行数据采集与回传项目  
采集 **位置、轨迹、海拔、姿态**，通过 **4G 实时回传** + **TF卡黑匣子存储**

![PlatformIO](https://img.shields.io/badge/PlatformIO-%23F5820D.svg?style=flat&logo=platformio&logoColor=white)
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

## 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/你的用户名/你的仓库名.git
cd 你的仓库名
