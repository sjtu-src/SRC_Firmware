## 简介
- 该仓库为搭载了 FreeRTOS 实时操作系统的SSL机器人嵌入式代码，适配上海交通大学SRC赛队自主开发的机器人 Version 1.5，主控芯片型号为STM32F407VET6。
- 该工程使用 STM32CubeMX 进行硬件资源配置与管理，使用 Keil uVision5 进行开发。
- 该工程使用 HAL 库进行开发，相较以往版本进行了较大改动，但其可移植性和可扩展性将得到极大改善。
- 目前代码仍在开发中，因此代码是**不完善的**，有 bug 为正常现象。

## 代码说明
主要代码存放在 .\Core 文件夹中，其中 .\Core\Inc 为头文件， .\Core\Src 为源代码。以下是各部分代码的功能说明：
- `main.c`：主程序。
- `robot.c`：机器人总体相关，如整机初始化、执行整机指令等。
- `motor.c`：电机相关。
- `pid.c`：电机控制相关的 pid 算法。
- `misc.c`：红外、各板载 LED 等杂项控制。
- `packet.c`：通讯解包相关。
- `comm.c`：通信相关。
- `action.c`：机器人运动相关。
- `simulate_i2c.c`：IO口模拟i2c通信。
- `param.c`：机器人相关常数配置。
- `error.c`：机器人异常状态标志。
- `NRF24L01.c`：通信主芯片NRF24L01驱动。
其他文件为基础的外设配置，此处不赘述。
---
## Introduction
- This repository contains embedded code for an SSL robot running on the FreeRTOS real-time operating system, compatible with Version 1.5 of the autonomous robot independently developed by Shanghai Jiao Tong University's SRC Team. The main control chip model is STM32F407VET6.
- The project uses STM32CubeMX for hardware resource configuration and management, and Keil uVision5 for development.
- The project is developed using the HAL library. Compared to previous versions, significant modifications have been made to greatly improve portability and extensibility.
- **The code is currently under development**, therefore it is **incomplete**. The presence of bugs is normal.

## Code Structure
Main code resides in the `.\Core` directory, with headers in `.\Core\Inc` and source files in `.\Core\Src`. Functional descriptions of key components:

- `main.c`: Main program  
- `robot.c`: Robot-wide functions (e.g., initialization, command execution)  
- `motor.c`: Motor-related operations  
- `pid.c`: PID algorithms for motor control  
- `misc.c`: Miscellaneous controls (infrared, onboard LEDs, etc.)  
- `packet.c`: Communication packet parsing  
- `comm.c`: Communication handling  
- `action.c`: Robot motion control  
- `simulate_i2c.c`: Software I²C implementation  
- `param.c`: Robot configuration parameters  
- `error.c`: Robot error status flags  
- `NRF24L01.c`: Driver for NRF24L01 communication chip  e  

Other files contain basic peripheral configurations and are not detailed here.

---
## 开发计划
- [x] 完成各硬件资源的初始化配置。
- [x] 完成机器人初始化部分代码编写。
- [ ] 确定操作系统线程分配及优先级配置。
- [ ] 完成自检模式
- [ ] 完成正常比赛模式

