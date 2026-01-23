# Sky-Masterminds-2 巡线小车

![Platform](https://img.shields.io/badge/Platform-STM32F103C8-blue)
![IDE](https://img.shields.io/badge/IDE-Keil%20MDK-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

基于 STM32F103C8 的智能巡线小车项目，采用双环 PID 控制架构，支持自动巡线和蓝牙遥控两种模式。

## ✨ 功能特性

- 🚗 **自动巡线模式**：位置环 + 速度环双环 PID 控制，精准跟踪黑线
- 📱 **蓝牙遥控模式**：通过蓝牙串口接收指令，支持前进、后退、左转、右转
- 🎯 **16路红外传感器**：高精度黑点检测，支持亚像素定位
- 🔄 **AB相编码器**：实时测量左右轮转速，闭环控制
- 🌀 **LSM6DSR六轴IMU**：陀螺仪辅助姿态估计，提高弯道稳定性
- 🛡️ **急停保护**：一键急停，确保安全
- 📺 **OLED显示**：实时显示运行状态
- 💡 **RGB状态指示**：不同颜色指示当前工作模式

## 🛠️ 硬件配置

| 模块       | 型号/规格      | 说明                         |
| ---------- | -------------- | ---------------------------- |
| MCU        | STM32F103C8T6  | ARM Cortex-M3, 72MHz         |
| 电机驱动   | 双路H桥        | PWM频率17KHz                 |
| 红外传感器 | 16路模拟       | 通过CD74HC4067多路复用器     |
| IMU        | LSM6DSR        | 6轴加速度计+陀螺仪 (SPI接口) |
| 编码器     | AB相正交编码器 | 左右轮各一个                 |
| 显示       | 0.96" OLED     | I2C接口                      |
| 通信       | 蓝牙模块       | UART2 (115200bps)            |
| 负压风扇   | 无刷电机       | PWM控制转速                  |

## 📁 项目结构

```
Sky-Masterminds-2/
├── User/                       # 用户应用代码
│   ├── main.c                  # 主程序入口
│   ├── PID_Controller.c/h      # 双环PID控制器
│   ├── Motor_ctr.c/h           # 电机驱动控制
│   ├── ABEncoder.c/h           # AB相编码器驱动
│   ├── BlackPoint_Finder.c/h   # 黑点检测算法
│   ├── BTComm.c/h              # 蓝牙通信协议
│   ├── LSM6DSR_Config.c/h      # IMU传感器驱动
│   ├── ADC_get.c/h             # ADC多路采集
│   ├── OLED.c/h                # OLED显示驱动
│   ├── RGB_Led.c/h             # RGB LED控制
│   ├── Key_Scan.c/h            # 按键扫描
│   ├── pose.c/h                # 姿态解算
│   └── Uart_Config.c/h         # 串口配置
├── Library/                    # STM32标准外设库
├── Start/                      # 启动文件和CMSIS
├── System/                     # 系统级驱动 (Delay等)
├── Objects/                    # 编译输出目录
└── DebugConfig/                # 调试配置
```

## 🎮 控制方式

### 按键功能

| 按键 | 功能                  |
| ---- | --------------------- |
| K1   | 启动/停止巡线模式     |
| K2   | 启动/停止蓝牙遥控模式 |
| K3   | 打印传感器状态 (调试) |
| K4   | 急停/解除急停         |

### 蓝牙指令协议

命令格式：`CMD:参数\n` 或 `CMD\n`

**全局命令** (任何时候都生效)：
| 命令 | 说明 |
|------|------|
| `ESTOP` | 急停 |
| `VON` | 负压开 |
| `VOFF` | 负压关 |
| `VS:xx` | 设置负压转速 (0-100) |
| `SPD:xx` | 设置基础速度 (0-100) |
| `?STATUS` | 查询状态 |

**遥控命令** (需先按K2进入蓝牙模式)：
| 命令 | 说明 |
|------|------|
| `FWD` / `FWD:xx` | 前进 (可指定速度) |
| `BWD` / `BWD:xx` | 后退 |
| `TL` / `TL:xx` | 左转 |
| `TR` / `TR:xx` | 右转 |
| `STOP` | 停止 |

## ⚙️ 控制架构

### 巡线模式 - 双环PID控制

```
                    ┌──────────────┐
   目标位置 ──────▶ │   位置环PID   │ ──────▶ 差速修正量
   (传感器中心)      │   (外环)      │
                    └──────────────┘
                            │
                            ▼
   ┌────────────────────────┴────────────────────────┐
   │                                                  │
   ▼                                                  ▼
左轮目标速度                                      右轮目标速度
   │                                                  │
   ▼                                                  ▼
┌──────────────┐                              ┌──────────────┐
│  速度环PID   │                              │  速度环PID   │
│   (内环)     │                              │   (内环)     │
└──────────────┘                              └──────────────┘
   │                                                  │
   ▼                                                  ▼
 左轮PWM                                           右轮PWM
```

### 蓝牙模式 - 仅速度环控制

蓝牙模式下禁用位置环，直接通过蓝牙指令设置左右轮目标速度，速度环PID保持稳定输出。

## 🔧 编译与烧录

### 环境要求

- Keil MDK-ARM v5.x
- ST-Link 或 J-Link 调试器
- STM32F10x标准外设库

### 编译步骤

1. 使用 Keil MDK 打开 `Project.uvprojx`
2. 选择 Target 1
3. 点击 Build (F7) 编译项目
4. 点击 Download 烧录到目标板

## 📊 状态指示

| RGB颜色 | 状态           |
| ------- | -------------- |
| 🟢 绿色 | 巡线模式运行中 |
| 🔵 蓝色 | 蓝牙遥控模式   |
| 🔴 红色 | 急停状态       |
| ⚫ 熄灭 | 待机           |

## 📝 开发说明

### PID参数调节

速度环和位置环PID参数位于 `PID_Controller.c` 中：

```c
// 速度环参数
SpeedPID_Param_t speed_param = {
    .kp = xxx,
    .ki = xxx,
    .kd = xxx,
    .output_max = 10000,  // PWM占空比上限
    ...
};

// 位置环参数
PositionPID_Param_t position_param = {
    .kp = xxx,
    .ki = xxx,
    .kd = xxx,
    .target_position = SENSOR_COUNT / 2,  // 目标位置为传感器阵列中心
    ...
};
```

### SysTick中断周期

系统Tick配置为 **2ms** 周期，用于：

- PID控制计算
- 编码器速度更新
- 传感器采样

## 📄 许可证

本项目采用 MIT 许可证开源。

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

---

⭐ 如果这个项目对你有帮助，请给个 Star 支持一下！

注：原项目地址https://gitee.com/purplesuner/stm32c8t6minicar
本仓库仅供团队备赛校赛使用
