# 主板代码迁移记录（2026-10-10）

作者：COEX、Xuanting Liu。

已按确认的 TIGERs 功率板 / POWER 电路迁移应用代码。用户已在 CubeMX 将 TIM9 CH1、CH2 改为 PWM mode 2 / High。未修改 `.ioc`、引脚复用、时钟、NVIC、外设初始化参数或已有任务配置；自动生成文件中的应用修改均在 `USER CODE` 区内。Keil 工程新增两个源文件。

## 修改内容

| 功能 | 实现 |
| --- | --- |
| 无源蜂鸣器 | TIM8 CH1，沿用 2 kHz 配置；开鸣为 50% 占空比，关鸣为 0%。原 `BEEP_ON/OFF` 调用保留。 |
| 拨码开关 | `GET_DIP_SW_D_IO()` 从旧 PC9 改为使用 `SW_D_GPIO_Port/Pin`，即当前 PA10。LED_COMM 原来已经使用生成的引脚宏，无须再次硬编码。 |
| 功率板 SPI | `board_monitor.c` 独占 SPI2，PB12 软件片选；ADC122S051 两通道各采 16 个值，丢弃切换后的前两个值，校验有效样本数量。沿用 TIGERs 温度和 251.42 V 满量程换算。 |
| INA226 | I2C3，7 位地址 0x40；检查器件 ID，配置连续采样、4 次平均。读取总线电压及有符号分流电压，按 2 mΩ 换算电流。 |
| SMB_ALERT | 配置 INA226 的低有效转换完成通知；中断只置标志，任务读 Mask/Enable 寄存器确认。任务也定期轮询，覆盖上电时 ALERT 已低的情况。当前未配置过流告警阈值。 |
| 充电控制 | 有效电池及电容测量后才允许充电；保持原目标 180 V，5 V 回差。60°C 停充、55°C 恢复；测量失败、低电压、踢球期间禁止充电。充电 3 秒未增加 5 V 时暂停 30 秒再重试。 |
| KICK_ERR / DONE | 保留生成的 `KICK_ERR` 名称；按 LT3751 DONE 的低有效、锁存语义处理，不当作“踢球结束”中断。低于目标且充电中出现 DONE，会进入故障重试等待。 |
| 踢球 | 修复单脉冲重启顺序，每次先停两通道、重置计数和预分频相位，再启动一个通道。力度 1–127 对应 100 μs–12.7 ms；零力度不触发，禁止两通道重叠，读数失效或极性错误时不触发。 |
| MOTOR5 | 保持 FG1 上升沿 EXTI 接线；ISR 只累加计数。按 1 对极、每转 3 个上升沿计算转速，100 ms 更新一次 PI 控制。合法非零档统一目标 5000 rpm，零档停止，保留正负命令的方向语义。FG2 保留输入，不当作正交编码器使用。 |
| PDN / KILL | 任务每 10 ms 检查 PDN，持续低电平 50 ms 后锁定关机请求，停止驱动、踢球、充电、蜂鸣器，再拉低开漏 KILL。若 USB 继续供电，每 100 ms 交替拉低/释放 KILL，其他任务不能重新启动负载。 |
| 遥测 / OLED | 移除 MCU ADC 读取。无线电压字节继续按旧协议比例编码并限制到 0xFE；OLED 使用实际电压，不再把遥测字节反算覆盖实际测量。 |

新增 `Update_Board` 应用任务在启动提示音和 IMU 初始化期间也运行，独占功率板 SPI2/I2C3，不占用 IMU/OLED 的 I2C2。栈为 384 words，创建失败进入现有 `Error_Handler()`。

## 参数与调试

`Core/Inc/cfg.h` 中可调整：

```c
#define DRIBBLER_POLE_PAIRS 1U
#define DRIBBLER_TARGET_RPM 5000.0f
#define DRIBBLER_SAMPLE_MS 100U
#define DRIBBLER_FG_TIMEOUT_MS 1000U
#define DRIBBLER_SPEED_KP 0.00003f
#define DRIBBLER_SPEED_KI 0.00006f
#define DRIBBLER_MAX_DUTY 0.55f
#define DRIBBLER_DUTY_RAMP_PER_S 0.5f
```

PI 增益是待实机调试的初值，不能据此保证实际电机已经稳定在 5000 rpm。最大占空比保持原代码的 55%；达到限幅后不会为追求转速继续增大输出。1 秒无 FG 上升沿会锁定吸球停机；必须先收到零档停止命令，再发送非零命令才能重新尝试。

调试接口：`dribbler_get()` 返回实际 RPM、目标 RPM、占空比和 `stalled`；`board_monitor_get()` 返回电压、电流、温度、有效标志、时间戳、充电和关机状态。电流正负取决于已沿用的分流电阻连接方向。INA226 的 Current/Power 寄存器未使用，电流由实测分流电压在 MCU 内换算。

## 找到并修复的问题

- `misc.c` 包含已经删除的 `adc.h` 并继续调用 `hadc1/HAL_ADC_*`。
- `i2c.c` 使用信号量却没有包含 `FreeRTOS.h`、`semphr.h`，造成 `SemaphoreHandle_t`、`pdTRUE` 等未定义。
- 新蜂鸣器仍被当作 GPIO，拨码数据仍读取旧 PC9。
- 踢球启动未重置计数器，原计算未正确计入 ARR+1；两路通道之间缺少运行互斥。
- 编码器方向保护的计数器只有 8 位，却需要计到 500，现改为 16 位。
- TIM12 ISR 的 `xHigherPriorityTaskWoken` 没有每次重置，并补充了信号量空指针检查。
- 吸球自测的空转死循环改为 `osDelay` 等待；删除不可达的 `break`。
- TIM11 的应用启动代码不再启动无用途的更新中断，PWM 继续使用原基础配置。

初始化期间也调用的蜂鸣器及电机输出函数保存/恢复 PRIMASK，避免在调度器启动前使用 FreeRTOS 临界区而遗留 BASEPRI 屏蔽。

## 验证及边界

- Keil ARMCC 5 实际编译、链接并生成 AXF/HEX，0 Error；1 Warning 是本机 Keil 的支持到期/授权提示。未更改或绕过授权设置。
- `python tests/test_board_migration.py`：传感换算、SPI 无效数据/失败、I2C 失败/恢复、测量过期、温度回差、充电故障重试、重复单脉冲、互斥、极性检查、遥测编码、5000 rpm 脉冲计数、PI 增功率/限幅、FG 丢失锁定、关机后禁止重新启动及启动中断屏蔽恢复均通过。
- 对比迁移前快照，自动生成区未变；TIM9 以用户本次重新生成后的版本为比较基准。
- 未连接实物烧录。模拟测试不能代替示波器确认踢球输出在上电、重复触发和脉冲结束后均正确，也不能代替电压/电流标定及 PI 负载测试。

参考本地 TIGERs：`src/shared/drv/adc_kicker.c`、`pwr_ina226.c`、`src/shared/module/power_control.c`、`kicker.c`。

外设语义核对：[STM32 RM0090](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f10x-microcontrollers-stmicroelectronics.pdf)、[A4931 数据手册](https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf)、[LT3751 数据手册](https://www.analog.com/media/en/technical-documentation/data-sheets/lt3751.pdf)、[INA226 转换完成标志说明](https://e2e.ti.com/support/amplifiers-group/amplifiers/f/amplifiers-forum/754969/ina226-when-conversion-ready-flag-is-set-and-clear-at-the-same-timing)。
