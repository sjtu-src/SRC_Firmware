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


## 2026-10-18 电机开机输出时序修复（历史方案，现行实现见下方）

依据用户 powerboard_schematic.pdf 第2–4页，确认四轮及吸球均为A4931，PWM低有效、BRAKEZ低刹车；各轮PWM/BRK已有5.1 kΩ上拉，Motor4使用RN20。

MX_TIM1_Init 在 USER CODE 2 调用 motor_prepare_pwm，保持BRAKEZ低、装载四路零CCR并UPDATE、检查四通道PWM启动结果后，才由MspPostInit切入AF。随后原init_motor释放刹车时，零输出已稳定为高。轮速PID仍等音乐/MPU自检完成后启动；start_motor不再重复启动HAL BUSY通道。未修改外设配置、引脚或自动生成区。

新增tests/test_motor_startup.py通过零输出建立/刹车顺序/预装载/IRQ保持/延后PID/五种HAL失败路径模拟，板级4S/6S回归及音乐测试通过。尚未烧录或实测；这修复软件时序缺口，不能替代验证只动右前轮的实际物理原因。详细证据和限制见根目录AGENT.md第17节。


### 2026-10-18 后续回归修正（取代上节的提前启动实现）

用户确认提前PWM版本消除轮动，但BAT由正常变为0。现已撤回MX_TIM1_Init中的motor_prepare_pwm调用，定时器初始化与HEAD一致；I2C自动恢复补丁也撤回，I2C驱动/采样与HEAD一致。启动阶段改为motor_hold_startup持续拉低BRAKEZ，保持硬件刹车，直到原start_motor时机才清零/UPDATE并启动四路PWM，之后释放刹车。未改变外设配置、I2C时序或运行时init_motor语义。

电机启动、4S/6S板级及音乐测试通过。该版本需实机验证BAT与轮动，不能把宿主通过或软件复位视作已找出根因。详见AGENT.md第20节。


## 2026-10-18 硬件失效与6S兼容性未决项

用户确认Q1损坏及测试电机支路保险丝熔断。原理图各电机支路SMAJ18A的持续工作电压18V、击穿20–22.1V与6S满电25.2V存在不匹配；需核对实装并优先排查TVS/功率桥短路，不能仅归为电机启动电流。

热插拔LTC4231的采样位置R7标为0Ω，若实装相同，其主动电流检测限流未建立；栅极R8/C4的缓启动仍在，不能替代短路限流。Q1需按SOA而非标题电流判断。当前软件配置6S不代表硬件6S兼容已验证。详细数据手册依据、用户观测和未决问题见AGENT.md第23节。本轮未改控制参数或保护。

## 2026-10-18 当前四轮启动方案：零PWM滑行

按用户要求替代启动期刹车：四轮BRAKEZ从GPIO默认值开始为高（.ioc同步），motor_hold_startup不再拉低刹车。MX_TIM1_Init先把PWM脚保持为GPIO高，即A4931关闭；保留原音乐/IMU之后的TIM1启动时机。motor_prepare_pwm完成四路CCR=0、UPDATE及PWM通道启动后，才调用HAL_TIM_MspPostInit切AF；失败时引脚保持GPIO高，电机滑行。

CubeMX重新生成后必须移除其在MX_TIM1_Init末尾重新插入的HAL_TIM_MspPostInit调用，保留USER CODE的GPIO高初始化。tests/test_motor_startup.py会检查这一时序。运行时控制、吸球启动及MODE3自动运动自检未修改。

启动模拟、4S/6S板级回归及音乐测试通过；Keil 0错误0警告，日志tmp/motor_startup_coast_build.log。未烧录或验证实机电流。用户已确认实物SMAJ18A并将其拆下，保险丝熔断时刻未知；Q1/熔断原因仍待硬件验证。详见AGENT.md第24节。
