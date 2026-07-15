# JY901S 姿态角显示（维特智能）

基于 **STM32F103C8** 的 **维特 JY901S** 九轴模块串口驱动示例：自动扫描波特率、解析角度帧，在 OLED 上以 **±xxx.x°** 显示 **Roll / Pitch / Yaw**（真实角度，1 位小数）。

---

## 功能概述

- 串口接收 JY901 协议（帧特征 `0x55`）  
- **自动波特率扫描**（`JY901_AutoBaud`），选出有效帧最多的波特率  
- 可选开启角度输出（`JY901_EnableAngleOutput`）  
- 显示使用 0.1° 整数内部存储，界面换算为真实度，避免误显示“好几百度”

---

## 硬件接线

| 模块 | STM32 |
|------|--------|
| TX | **PB11**（USART3_RX 等，以驱动为准） |
| RX | **PB10**（可选，用于发配置） |
| GND / VCC | 共地 / 供电 |

---

## 目录结构

```
├── User/main.c
├── Hardware/
│   ├── JY901S.c / .h     # 初始化、自动波特、解包
│   └── OLED.*
├── System/ Library/ Start/
└── Project.uvprojx
```

### 主要 API

```c
JY901_AutoBaud();           // 扫描并选定波特率
JY901_EnableAngleOutput();  // 打开角度输出
JY901_Parse();              // 主循环解包
// 数据：JY901.roll_x10 / pitch_x10 / yaw_x10  （单位 0.1°）
```

---

## 使用

1. 编译下载，上电显示 `Scan baud...`  
2. 显示选定波特率后进入姿态界面  
3. 转动模块，观察 R/P/Y 变化  

无数据时检查接线、供电，以及模块是否配置为串口连续输出角度。

---

## 相关仓库

- [lingke-6axis-gyro](https://github.com/kaka12331/lingke-6axis-gyro) — 另一类 6 轴串口 IMU  
- [mpu6050](https://github.com/kaka12331/mpu6050) — I2C MPU6050 + DMP  

用于学习与备赛交流。
