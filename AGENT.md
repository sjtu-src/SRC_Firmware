# SRC RoboCup SSL 下位机：项目背景、TIGERs 对比与持续工作记录

## 固定源码路径（用户于2026-10-20再次确认）

- **TIGERs嵌入式固件目录：`E:\SRC\Tigers\Firmware-master\Firmware-master`。** 后续用户提及“TIGERs固件”时，默认直接读取该目录的实际源码，不再询问源码位置。
- 本次会话此前的固件对比实际读取的就是此目录；“本地参考固件”指该目录，不是另一个未知位置。应以具体文件、函数和实际代码说明结论。
- 路径确认不等于确认该解压源码的Git提交号或现役比赛版本；这些证据边界与固定路径分别记录。

更新日期：2026-10-20（Asia/Shanghai）。本文件是项目长期知识入口；自动化协作入口见同目录 AGENTS.md。

## 1. 项目背景与证据范围

- SRC：上海交通大学 SRC 队的 RoboCup Small Size League 足球机器人下位机；用户说明以 TIGERs 为参考开发，主控 STM32F407VET6。
- SRC 根目录：E:/myCode/STM32/SRC_Firmware；主要应用 Core/Inc、Core/Src；Keil 工程 MDK-ARM/SRC_Firmware.uvprojx；STM32CubeMX + HAL + FreeRTOS。
- README 提及 NEUMANN_S01、兼容乐博旧车；当前源码已加入新功率板、FG 测速、MPU6050 和航向控制。README 不能替代当前编译分支与实物配置。
- 本次 SRC 基线提交：3da289be3a8ff9108b107bfc0017a5076ee28448，2026-10-16 18:40:34 +0800，提交说明 delet some unuseful files。分析开始时 git status --short 无输出。
- TIGERs 本地参考：E:/SRC/Tigers/Firmware-master/Firmware-master；README 明确面向 v2020 机器人，包括 main2019、motor2019、ir2019、基站与 bootloader。这是解压源码目录，本次未取得其可靠 Git 提交号；不能认定与 TIGERs 当前比赛版本一致。
- 本地 TDP：E:/SRC/TDPs/TIGERs Mannheim；有 2011、2013—2020、2022—2024 及 2021 冠军论文，缺 2025/2026。本次网络可访问，直接阅读官方 2020、2022、2024 相关章节及 2025/2026 补充内容，未声称逐页读完所有历史论文。
- 比较对象是下位机控制实现。没有读取 SRC 上位机全工程、TIGERs Sumatra 全工程、SRC 完整原理图和机械 CAD；无法据此判断整队竞技实力、机械等价程度或现役比赛表现。
- 本次只增加分析与协作说明，不修改控制代码、不烧录、不提交 Git。
- 证据约定：“已实现”指本次源码与调用路径可见；“TDP”指该年份公开设计；“推断/待验证”需要实测或额外硬件资料。默认参数可能被 TIGERs Flash 配置覆盖。

## 2. 主要差别及影响排序

总判断：SRC 已具备四轮编码器闭环、IMU 相对航向反馈、吸球转速闭环、电源监测和保护基础。TIGERs 本地版本进一步把电流/转矩、车体状态估计、轨迹、持球力估计与运动约束贯通。最大差距在反馈量、控制分层和标定，而非简单“PID 参数不够好”或只差主控算力。

| 维度 | SRC 当前实现 | TIGERs 本地实现 | 影响 |
| --- | --- | --- | --- |
| 控制平台 | F407，FreeRTOS，主控输出 PWM/DIR/BRK | H7 主控、ChibiOS，独立 STSPIN 电机控制器、IR 处理器 | 分担高频电流环，便于统一反馈和记录；不能只按 MCU 主频比较性能 |
| 驱动电机 | 编码器速度/位置增量 PID；PWM 电压模型限幅 | 独立驱动 FOC，D/Q 电流控制约 20 kHz，速度环和转矩前馈 | 实际转矩可控、加减速一致性与抗扰动潜力差异大 |
| 吸球执行 | FG1，100 ms 窗口，10 Hz PI，固定 5000 RPM | 六步换相；1 kHz 速度外环 + 40 kHz 电流内环；速度估计 | 接球冲击、低速持球和受力稳定性是主要差距 |
| 球状态 | 主要是红外有球/无球；吸球 PI 不读取该状态 | 电流、转速、摩擦标定推算力，结合 IR 与球观测形成状态机 | 能区分“球在嘴边”和“已可靠持球” |
| 姿态/定位 | MPU6050 100 Hz 四元数互补估计、相对 yaw；无车载全局位姿融合链路 | 陀螺仪、加速度、编码器与视觉的 5 状态 EKF/INS，延迟缓存 | 长时方向一致性、视觉遮挡、全局轨迹跟踪差距大 |
| 运动更新 | 轮速任务约 2.002 ms；车体航向随命令包更新；XY 每包限速增量 | 1 ms 统一状态估计、技能、轨迹、控制、输出 | SRC 车体闭环受丢包和通信抖动影响 |
| 模型/前馈 | 轮子几何、减速比、经验总增益 3/8 | 摩擦/效率、编码器动态模型及逆补偿、加速度转矩前馈、载荷分配 | 高动态响应和不同车一致性 |
| 持球运动 | 固定 XY 增量限制，不随持球质量调整 | 根据达到的持球力调整横向与旋转加速度能力，平滑释放球 | 转弯/急停丢球与持球稳定的关键系统差异 |
| 调试 | OLED、无线姿态/轮速遥测、已有宿主回归 | 状态/参考/输出/性能日志，电机采样模式、参数文件、测试与标定入口 | 定位问题与批量维护效率 |

不能将 SRC 的“PWM 25 kHz”当成控制环 25 kHz；不能将 TIGERs 的 40 kHz 电流环当成整车闭环 40 kHz。比较必须在相同控制层级。

### 差距如何衡量

不能从源码给出“差几年”“落后百分之多少”或“达到冠军多少水平”。按功能层级，SRC 的基础轮速与相对航向控制已经存在；在吸球力闭环、延迟融合/全局轨迹、持球与运动协同方面，缺的是完整功能链路，并非一次调参即可补齐。

先建立相同场地、球、轮胎、电池、车速下的基线。以下是建议测试，不是既有测量结果，也不是 TIGERs 官方指标：

| 场景 | 建议记录 | 比较方法 |
| --- | --- | --- |
| 直线与侧移 | yaw 峰值/RMS、横向偏差、四轮目标/反馈 | 多方向、不同速度、不同电量重复，分开几何误差和航向误差 |
| 原地/复合转动 | 外部视觉角速度与 IMU、角速度阶跃超调/稳定时间 | 验证旋转单位、符号和传感器饱和，覆盖平移+旋转 |
| 静态漂移 | 60 s / 300 s 外部航向误差、温度、零偏 | 外部参考评估 yaw 漂移，不用其自身积分值充当真值 |
| 接球 | 多档实际来球速度、成功率、回弹距离 | 正面/偏心接球分别重复，记录空转/接触时转速及负载 |
| 带球 | 最大可靠侧向加速度、角速度/角加速度、急停丢球率 | 以可重复轨迹确定能力包络，逐档提高，避免先用算法假定能力 |
| 抗扰/释放 | 外力扰动后恢复、停吸后脱球行为 | 同时记录红外、RPM、占空比、独立电机电流（若具备） |
| 调度/通信 | 控制 dt、IMU 数据年龄、命令年龄、丢包、截止期超时 | 报告分位数与最差值，平均周期不足以说明实时性 |

## 3. 必须优先核验的具体问题

### P0：2026-10-16 基线的车型宏问题（当前状态见下）

2026-10-18 复核：当前 d76a6f1 工作树已定义 `TT_0=4`，因此“不定义 TT_0 导致落入 LEBO”不再适用于当前源码；下文保留为旧基线证据。TT 分支仍写 `NEW_DRIB_MOTOR`，与已定义的 `NEW_DIRB_MOTOR` 不同，需单独核对，本次未修改车型配置。

证据：Core/Inc/cfg.h:31 写 ROBOT_VERSION TT_0；TT_0 在所查源码及 Keil Define 中未定义。LEBO_ROBOT=0，且该分支出现在 TT_0 分支之前。

本次将 cfg.h 仅移除 include 后送 GCC 预处理，实际得到：
- INFRA_TYPE OLD_INFRA
- MOTOR_TYPE OLD_MOTOR
- ENCODER_TYPE OPTICAL_ENCODER
- DIRB_MOTOR_VERSION OLD_DIRB_MOTOR
- DRIB_DIR OLD_DIR
- MOTION_COMMAND_GAIN 3.0f

原因：C 预处理条件中的未定义标识符按 0 处理，TT_0 因而先匹配 LEBO_ROBOT。不是自动进入新加的 TT 分支。后者还有 NEW_DRIB_MOTOR / NEW_DIRB_MOTOR 拼写不一致，修复 TT_0 定义后也需继续检查。红外极性、吸球方向等会因此与预期不同；只有确认实物接线后才能定最终取值。本次未擅自修改车型。

### P1：编码器计数与物理单位需贯通核验

Core/Src/motor.c:update_encoder 使用硬件四倍频计数计算 cur_speed，位置则除以 4；action.c:do_move 使用 ENCODER_COUNTS_PER_TURN_SET 转换目标。不能仅看“1000 线”注释判断它代表线数还是四倍频 counts。实测每电机轴/轮轴一整圈计数，再核对 LIU_WANG_CONST、轮径、符号与 MOTION_COMMAND_GAIN。

MOTION_COMMAND_GAIN 对平移、旋转目标一并生效；3 和 8 已进入航向参考公式，却仍需用实测角速度/轮速确认物理映射。不能据静态源码直接断言存在 4 倍误差，也不应继续用经验增益掩盖单位问题。

### P1：TIM7 计时频率与轮速换算还存在两倍不一致

按当前 HSE_VALUE=25 MHz、PLL M=25/N=336/P=2、AHB 不分频、APB1 四分频配置，HCLK=168 MHz，APB1=42 MHz，TIM7 输入为 84 MHz。tim.c:349 的 Prescaler=20，实际计数频率为 84 MHz/(20+1)=4 MHz。motor.c:237 却用 ENCODER_TIM_CLK_FREQ*2=2 MHz 去计算 counts/s。因此，在当前时钟配置与晶振频率成立、计数器未溢出的条件下，发布的轮速只有按计时器真实时间计算值的一半。

此项与编码器四倍频的单位问题是两件事，应分别检查；不能互相抵消后继续用经验增益。先核实实物晶振与运行时 RCC/PSC，再统一计时公式并重做速度/航向标定。参考 [ST RM0090](https://www.st.com/resource/en/reference_manual/DM00031020.pdf) 的 RCC 定时器时钟与 TIM6/7 预分频说明。本次未改外设配置或换算公式。

### P1：车体反馈与网络收包耦合

comm.c:do_packet_process -> on_robot_command -> do_acc_handle_move -> HeadingControl_Update；没有独立周期性车体航向控制调用。无线暂时无新包时，轮速 PID 继续追踪上次目标，航向控制不会用新 IMU 自动重算。IMU 的 100 ms 新鲜度检查也只在调用时执行；通信超时路径约 500 ms 才主动清零。

建议固定周期控制任务只读取“最新命令快照”，通信负责更新命令/时间戳；让故障降级、参考积分、航向误差和加速度约束均随实际 dt 执行。

### P1：MAX_ACC 是每包速度增量，不是 m/s²

action.c:do_acc_handle_move 直接限幅 speed_x/y 与上次值之差，未除以 dt。实际加速度受包率、总增益和整数 ceil 取整影响。不要把 MAX_ACC=10 写成已知的 10 m/s²。建议统一 SI 单位、按 dt 限幅，并测试正负方向的取整偏差。

### P1：驱动“转矩限制”不是实测电流闭环

motor.c:do_update_motor 向 pid_step 传固定 16.0f；pid.c:pid_step 的 calc_max_output 也再次固定 16.0f，未按传入 bat_v 使用。cfg.h 默认电池已是 6S。它依据 R、Kv、Kt 的静态电压模型约束输出，不能保证实际转矩准确跟随，无法与 TIGERs 电流闭环等同。

### P1：IMU 量程与参考命令范围不一致

MPU6050 配置 ±500 deg/s。当前增益为 3 时，航向参考每单位为 0.025*3 rad/s，参考命令绝对值约超过 116 即超过 500 deg/s；若增益 8，则约超过 44 即超过量程。这是“参考计算超量程”的判断，实际机器人能否达到该角速度还取决于正确的编码器单位和执行能力。应检查原始读数削顶、限制可实现参考或按硬件能力选择量程，不能盲目仅增大传感器量程。

### P2：文档与遗留宏容易误导

MPU6050_GYRO_USED=0 在当前所查应用源码没有作为新链路的开关；HEADING_CONTROL_ENABLE=1 才是航向控制开关之一。不能据旧宏说“未用陀螺仪”。
MDK-ARM/ATTITUDE_TELEMETRY.md 仍有“无运动校正”“任务优先级更低”等早期描述；当前源码已接入航向，IMU 为 AboveNormal。优先相信源码、最新 HEADING_CONTROL.md 和实际构建。本次在本文件指出历史差异，未重写历史记录。

## 4. TIGERs 如何控制吸球并提高稳定性

### 4.1 电机底层：速度目标与转矩上限同时下发

主要证据：
- TIGERs src/main2019/robot_impl.c:431，RobotImplDribblerOutput。
- src/motor2019/dribbler.c，DribblerMain：每 1 ms 执行控制。
- src/motor2019/dribbler_ctrl.c，DribblerCtrlUpdate 与 DribblerCtrlPerPWMCycle。

执行链为：
滚筒目标线速度/接触力 -> 换算电机角速度及转矩 -> I_limit=torque/Km -> 速度控制器输出 I_set（受 I_limit 限制） -> 40 kHz 电流 PI -> 电压/PWM -> 六步换相 BLDC。

驱动轮 FOC 与吸球电机六步换相是不同实现，不能混为一谈。速度设定不是必须维持的硬约束；接触负载增大时速度可以下降，重点在电流约束下维持可控作用力。

电流每 PWM 周期采样，停机期间估计零点；用滤波后的输出电压符号辅助电流符号处理。电流控制输出按电源电压限制；PI 实现含限幅/抗积分饱和。不是读取整车耗电后推算一个经验 PWM。

### 4.2 无吸球编码器时仍能在低速更新速度估计

DribblerCtrlUpdate 使用平均施加电压和测得电流：
omega_est ≈ (U_avg - R*I_avg) / Ke。

40 个 PWM 周期组成 1 ms 平均，速度估计不必等 Hall 边沿。在低速和近停转时，这比仅等稀疏 Hall/FG 更适合其控制方式，但结果依赖 R、Ke、电压与电流标定。它不是无误差的实际转速传感器，参数温漂等仍会影响结果。

### 4.3 从“检测有球”升级到“估计持球力”

src/robot/fusion_ekf.c:228 附近：
1. 滤波吸球电流。
2. 利用空载电流—速度查表估计摩擦项。
3. 用电磁转矩减去摩擦转矩，按传动比和滚筒半径换算接触力。
4. 力达到门槛并且红外遮挡，判为 ACTIVE_DRIBBLING；只有遮挡则为 AT_BARRIER。
5. 根据达到目标力的比例做 strong/weak 判定与迟滞；默认 strong-on 70%、off 40%，持续约 100 个 1 ms 周期才稳定归类 strong。
6. 还存在 FAR_DETECTION、DEAD_RECKONING、UNKNOWN 等球状态。

这仍是模型估计力，不是安装力传感器直接测得。电流含摩擦/惯性与测量误差，需要标定。参数及条件不能脱离代码直接照搬。

### 4.4 吸球力与车体运动协调

src/robot/ctrl_panthera.c:176，controlDribbler：
- 按球在滚筒坐标系中的相对速度方向和幅值形成力目标，不是仅按机器人车速线性加 RPM。
- ACTIVE_DRIBBLING 且实际/目标力低于 0.8 时，逐步增加滚筒目标速度并设置上限；其他情形逐步回落至输入目标。
- 达到持球力不足时，收紧对应横向和旋转加速度约束，降低甩球风险。
- 停吸有力和速度递减的释放过程；吸球温度有独立反馈，robot_impl.c 中 80°C/70°C 为默认过热迟滞。
- 这些逻辑作用于轨迹生成前的限制，因此吸球能力参与决定“机器人允许怎么动”。

稳定性来自执行环、接触力估计、球状态、运动限幅、机械接触与阻尼的配合，不是任何单一 PID 能保证。

### 4.5 机械设计与历史 TDP 的对应关系

2020 硬件论文记录了当年的吸球、传动、红外与阻尼设计；2022 §1.1 进一步强调双自由度结构及上下阻尼改善接球与持球。2024 §2 描述低速两点接触配合转矩控制，可在负载下接近停止，同时承认居中和带后旋踢球方面的取舍。不能把某一年结构、某一年滚筒直径和本地仓库默认值拼成一台未经确认的“最新机器人”。

SRC 是否具备同样的阻尼、滚筒材料、接触角度、行程和铲子间隙，本次资料不足，需机械组实物核查。软件不能补偿所有接球回弹问题。

### 4.6 SRC 当前吸球实现与直接差距

Core/Src/dribbler_control.c：
- 合法命令范围 -3..3，所有非零档只保留方向，目标统一 5000 RPM；档位大小没有对应不同吸球力度。
- FG1 上升沿计数；假设一对极、每电周 3 个上升沿。
- 100 ms 更新：RPM=edges*60000/(3*pole_pairs*elapsed_ms)。5000 RPM 时约 25 边沿/窗，1 边沿对应约 200 RPM；在更低速时估计相对误差/延迟更明显。
- PI 带条件积分、0..55% 输出限制，上升斜率 0.5 duty/s；Board 任务 10 ms 调用，但核心计算仍由 100 ms 条件节流。
- 1 s 无 FG 锁定停机，需先下发零档清除锁存。
- PI 没有读取红外、持球力、电机相电流或车体运动，不区分接球/持球/释放的控制目标。

板上 INA226 用于电源总线电流监测；未见该值作为 MOTOR5 独立绕组电流反馈进入吸球控制。不能拿总电流直接替代电磁转矩闭环。当前外部 A4931 控制接口也不同于可编程 STSPIN 控制器；是否能增加独立采样/限流，应查原理图和驱动能力。

与 TIGERs 近停转仍可持球相比，SRC 的 1 s 无 FG 会停机；若以后采用低速转矩持球，必须重新区分“可控静止”和“异常堵转/反馈丢失”。这不是建议现在去掉保护。

建议先恢复有意义的多档目标并记录负载下 RPM/占空比/红外，再在具备独立快速电流采样和可控驱动的条件下引入电流/转矩约束。仅把 FG 更新窗口缩短，或只调高 PI，不会得到同类持球力控制。

## 5. TIGERs 陀螺仪与运动闭环

### 5.1 传感器、标定与固定周期

src/main2019/dev/imu.c + src/shared/drv/imu_icm20689.c：
ICM20689 经 SPI 采样，IMU 任务约 1 ms。src/main2019/robot_impl.c:198 附近读取标定参数，减去 gyro bias 和加速度偏置/倾斜偏置，保留时间戳。

src/robot/robot.c:107，RobotTask：
每 1 ms 顺序更新传感器、状态估计、技能、控制、执行器、日志与反馈；与新网络包是否刚到达分离。性能时间也被记录，可检查实时负载。

### 5.2 估计层：当前角速度与融合航向职责不同

src/robot/fusion_ekf.c:80，FusionEKFUpdate：
- 5 状态为平面位置 x/y、航向 theta 和局部平移速度 vx/vy；输入为 gyro-z、acc-x、acc-y。不是完整 3D 姿态 EKF，也没有在这 5 状态中显式估计陀螺仪 bias。
- 视觉提供 x/y/theta 观测，按传输延迟 + 捕获延迟写入历史时间槽。
- 延迟融合窗口执行 EKF，INS 在当前时间向前传播，并由历史融合结果进行平滑修正；不可简化为“把延迟视觉当当前值融合”。
- 编码器经运动学与动态模型提供平移速度。在无视觉时退化为编码器/陀螺仪里程推算；不能说失去视觉仍不会漂移。
- 旋转里程推算在编码器角速度很小时保留编码器值，运动时使用陀螺仪。
- 输出 pState->vel[2] 直接是当前 gyro-z，而非视觉航向差分；航向则来自融合状态。
- 虽读取磁力计，该 EKF 的测量链路未使用磁力计作绝对航向观测。不能归因于“指南针纠正 yaw”。

robot_impl.c 默认 visCaptureDelay=20 ms、fusionHorizon=35 ms、visionTimeoutMs=1000；均可能由持久化配置覆盖，不能把这组默认值当现役所有机器人实测延迟。

### 5.3 控制层：轨迹前馈 + 航向误差 + 实测角速度反馈

ctrl_panthera.c:137，controlW 的主要结构（简化，原代码另有模式、限幅和增益调度）：
omega_out = omega_traj
          + scale * [ Ktheta*(theta_traj-theta_est) + Komega*(omega_traj-gyro_z) ]。

局部速度模式同样用 gyro-z 做角速度反馈；全局角度模式额外使用融合航向误差，近目标抑制抖动。不能笼统称它是单一“角度 PID”或说所有模式都带角度外环。

controlXY 对全局位置误差进行反馈并转换到局部坐标，同时补偿旋转影响。CtrlPantheraUpdate 还有轨迹生成、摩擦/效率补偿、编码器动态逆补偿、加速度转矩前馈及轮载荷分配。torqueVectoringStrength 的默认值因直驱/减速配置不同，不能说所有变体默认都启用相同强度。

最终输出轮速和转矩，底层按模式执行速度/电流控制。因此陀螺仪校正的是车体实际旋转，编码器闭环则解决各轮执行误差；两层反馈相互配合。

## 6. SRC 当前陀螺仪闭环

### 6.1 已经真正接入，不能误判为仅显示姿态

调用链：
MPU6050_ReadMotion -> Update_Imu / ImuFilter_Update -> g_robot.attitude
-> 收到运动命令 -> do_acc_handle_move / HeadingControl_Update
-> do_move 四轮运动学 -> pid.set -> do_update_motor 编码器 PID -> PWM。

Core/Src/freertos.c 中 ImuUpdate 为 AboveNormal，vTaskDelayUntil 10 ms；sensor output 为 100 Hz，DLPF 20 Hz，量程 ±500 deg/s / ±4g。与 OLED 共用 I2C2，有 mutex，但总线竞争和阻塞仍需测最差样本年龄。

imu_attitude.c：
启动要求至少 200 样本/2 s 静止，检查加速度模长、gyro 与方差，估计静态零偏；归一化四元数积分，用重力矫正倾斜；过强线加速度时拒绝重力校正。读错或采样间隔 >100 ms 会失效并重新要求静态校准。

它不提供绝对 yaw。加速度计的重力只约束倾斜，绕重力方向的 yaw 仍会漂移。当前这块板 Z 轴加速度有两面标定系数，不能照搬到其他 MPU。

### 6.2 运动状态下的航向策略

Core/Src/heading_control.c:35：
- 正常/Cray 模式 + SPEED_PID + IMU_VALID + 样本年龄不超过 100 ms，才进入辅助控制。
- HEADING_SENSOR_YAW_SIGN=-1，将陀螺仪方向转换到现有车体命令约定；与遥测原始符号区分。
- 有旋转命令时，以 0.025 rad/s * MOTION_COMMAND_GAIN 换算期望角速度，再积分成相对 yaw 目标；角度误差形成期望角速度修正，角速度 PI 在保留原命令前馈的基础上给附加修正。
- 有平移、旋转命令为 0 时，用 yaw P+I 与 -gyro-z 阻尼保持捕获的方向。
- 目标误差限 ±30°，附加修正限 ±32 命令单位，积分限幅；并非控制能力无限。
- 整体全零命令会 Reset 并返回 0，因此停车不主动抵抗外部扭转；不能将该行为描述为“始终定向”。
- 无效/过期 IMU 直接保留原旋转指令降级；位置 PID 和非驾驶模式不使用这层航向。
- 更新超过 100 ms 会重新对齐目标，避免把丢失时间直接积分。

这已包含有意义的相对航向和角速度反馈，差距并非“没有陀螺仪 PID”。但没有下位机视觉航向校正、全局位置状态、带延迟估计或统一车体控制周期。

### 6.3 与 TIGERs 的直接差距

1. SRC 的航向长期基准靠 gyro 积分；TIGERs 可借视觉 theta 约束漂移。
2. SRC 反馈随收包；TIGERs 固定 1 kHz 循环。100 Hz IMU 与受 DLPF/通信影响的总延迟限制 SRC 可调带宽，不能照抄 1 kHz 参数。
3. SRC 平移基本是轮速闭环；没有从融合车体位置/速度到本地轨迹的相同状态链路。四轮都达到目标不等于车体不打滑。
4. SRC 不会根据持球质量改变旋转/侧向轨迹能力；gyro 单独抑制 yaw 不能防止侧向甩球。
5. SRC 位置 PID 指各轮编码器位置目标，不等同于全局场地 x/y/theta 位置闭环。
6. F407 的算力不是移植基础 1 ms 控制、平面估计的充分否定理由；先测 CPU 占用、调度和总线耗时。40 kHz 电流环需要对应驱动/ADC/PWM 时序能力，不应将所有 TIGERs 代码搬入主控。

## 7. 分阶段建议

按风险和收益推进，具体修改需另一次实现任务；本次不替用户选实物配置。

P0：确认 TT_0 与吸球型号宏、红外极性；核实每轴一圈计数、方向、减速比、轮径；确认软件输出对应实车。保留实测记录，做到单位从协议到轮轴再到陀螺仪一致。

P1：把车体航向与运动约束从收包函数移到固定周期任务，建立命令/样本时间戳、超时与有效标志；按 dt 限加速度，明确零命令停车与驻车保向需求。先验证当前 IMU 与总线时序，再决定是否提高采样率。建立可下载日志，而非只看 OLED。

P1：在不改变底层驱动的阶段，保留保护，记录 FG、PWM、球检测、命令、真实 dt，做接球与带球能力试验。可设计接球/持球/释放目标，但未实测前不能假定它就等价于转矩控制。

P2：核对 MOTOR5 驱动电路与独立电流采样可能性，选择可控限流/转矩方案，完成空载摩擦、R/Ke/Kt 和温升标定；再加入持球力估计、强弱状态及带球运动限幅。轮电机是否升级独立 FOC 同样取决于硬件接口。

P2：扩展上位机协议，下发视觉 x/y/theta、采样时间/年龄和置信度；先验证 yaw 对齐和延迟，再做平面融合、轨迹与前馈。需处理角度跨 ±pi、视觉跳变、丢失重连；不要一开始直接堆完整 EKF。

P3：以每车参数与可复现数据维护能力包络，逐步引入摩擦补偿、动态前馈与技能协同。目标是让规划使用机器人真实可执行的能力。

## 8. TDP 阅读地图与公开来源

- [SSL TDP 索引](https://ssl.robocup.org/team-description-papers/)。
- [TIGERs 官方年度索引](https://www.tigers-mannheim.de/publications/tdps/)。
- [2020 ETDP](https://download.tigers-mannheim.de/tdps/soccer_smallsize__2020__TIGERs_Mannheim__0.pdf)：重点 §2.3、§3，硬件结构、电机控制器、IMU 与视觉输入背景。
- [2022 ETDP](https://download.tigers-mannheim.de/tdps/soccer_smallsize__2022__TIGERs_Mannheim__0.pdf)：重点 §1.1，机械阻尼演进；其余主题主要为上位机调试/测试。
- [2024 ETDP](https://ssl.robocup.org/wp-content/uploads/2024/04/2024_ETDP_TIGERsMannheim.pdf)：重点 §2.1/§2.2，吸球接触与级联控制；与本地 dribbler_ctrl.c 交叉核对。
- [2025 ETDP](https://download.tigers-mannheim.de/tdps/soccer_smallsize__2025__TIGERs_Mannheim__0.pdf)：主要为传球候选生成、过滤、评分、执行；不是新的下位机电流环论文。
- [2026 ETDP](https://download.tigers-mannheim.de/tdps/soccer_smallsize__2026__TIGERs_Mannheim__0.pdf)：主要为球运动模型、标定与比赛中验证，以及基于视觉估计运动能力；应借鉴数据标定流程，不能据它假定本地固件已包含所有新功能。

2025/2026 已在线查阅，未下载覆盖用户本地资料库。有关电流内环等具体实现，本报告主要依据本地可核对代码；论文用于解释设计目的和演进，避免把论文设想误当现有代码。

## 9. 2026-10-16 本次工作与验证记录

完成：
- 阅读 SRC 配置、通信/命令、运动学、轮速 PID、IMU、航向控制、吸球与板级监测关键链路。
- 阅读 TIGERs 机器人循环、robot_impl、fusion_ekf、ctrl_panthera、驱动/吸球电机控制、IMU 驱动及默认配置。
- 结合已有 HEADING_CONTROL.md、BOARD_MIGRATION.md、ATTITUDE_TELEMETRY.md 区分历史说明与当前实现。
- 在线核对 2020/2022/2024 核心设计，以及 2025/2026 新主题。
- 根据当前 HSE/PLL/APB1/TIM7 配置与 ST RM0090，推导并记录轮速计时系数的两倍不一致；待核实实物时钟。
- 以 GCC 仅预处理 cfg.h（移除硬件 include）验证车型落入 LEBO 分支；未把此操作当成完整固件构建。
- 运行已有 python tests/mpu6050/test_heading_control.py：通过（gain-aware reference、复合运动、hold、bypass、stale）。
- 运行已有 python tests/test_board_migration.py：通过，脚本覆盖 4S/6S 两配置的 power/charge/pulse/FG/shutdown/discharge/故障与超时等逻辑。
- 建立 AGENT.md 与根目录 AGENTS.md 协作入口。

限制：
- 宿主测试使用模拟硬件，不能证明接球成功率、实际 PID 带宽、独立限流、传感器真实极性或车体运动精度；现有航向测试提取部分宏，不能替代全工程配置验证。
- 未做 Keil 全构建、TIGERs 构建、实物烧录、示波器测量、场地实验；已有文档中的旧编译成功记录不是本次构建结果。
- 初始 Windows 沙箱执行器初始化失败，本次只读分析、已有测试及授权文档写入使用工具的沙箱外执行路径。没有修改固件参数。
- 不以代码数量、主频比例或论文年份推算 SRC 的竞技差距。

## 10. 后续维护约定

后续重要认识、改动、验证、工作结果应在任务结束前追加，必要时修订上文；不要把建议变成既成事实。

每次记录：
- 日期与源码基线/分支；
- 用户目标、实际改动与文件；
- 新发现及证据（函数/文件/行号、论文年份/章节、实验数据）；
- 验证命令、结果与硬件条件；
- 已知限制、待办与被取代的旧结论。

优先级：用户当前明确要求 > 适用 AGENTS.md > 本文件已有结论；实时源码/实测证据优于旧说明。实物参数、引脚、控制增益、协议字段或上位机行为改变时，应更新相关专题文档和本文件。未经验证，不宣称“稳定吸球”“绝对航向无漂移”“实现了电流闭环”。


## 11. 2026-10-17 MODE3 开机音乐

- 基线：分支 TEST_TIGERs_Archetecture，提交 2245581e24d732d1969edf2b6c689e8e04b10931；开始时工作区干净。
- 用户要求：将自检模式 MODE3 开机蜂鸣器改为所附《小猪佩奇》谱例，可独立使用 music.h。
- 实际改动：新增 Core/Inc/music.h，按用户图片第一页片头第 1–12 小节制作单声部简化音符表（C 调、96 BPM、十六分音符时值，省略伴奏并简化密集装饰音）。并非整首六页曲谱或原声录音。
- Core/Src/freertos.c：仅 SELFTEST_MODE=3 用 Play_Mode3Music 替换原四声模式提示，播放一次约 15.6 秒；其他模式保持原提示。音乐结束后继续原有 1 秒间隔、MPU6050 检测和检测结果提示，因此自检动作开始相应延后。
- 沿用 PC6/TIM8 CH1、现有预分频和 50% PWM，由 APB2 时钟计算音符周期；结束/中断恢复原 ARR（当前 499，即 2 kHz）并静音。不修改引脚、车型、控制增益、协议、外设初始化或任务配置。
- 继续经过 set_buzzer 的 board_actuators_disabled 保护，每个音符前也检查禁用状态，关机期间的 Board 任务仍可运行。
- 验证：python tests/test_board_migration.py，4S/6S 全部通过；tmp/mode3_verify.py 提取实际播放函数，以 GCC -Wall -Wextra -Werror 编译模拟，验证 15600 ms / 73 个有声音符、恢复 ARR、最终静音和关机中断；git diff --check 通过。模拟文件位于忽略的 tmp 目录。
- 限制：未烧录、未实机试听或示波器测频；图片为六页中的第一页，此实现是该页片头的蜂鸣器简化版，不宣称完整复现钢琴伴奏或所有装饰音。宿主模拟不替代硬件验证。
- 完整构建：E:/Keil_v5/UV4/UV4.exe -b SRC_Firmware.uvprojx -o tmp/mode3_build.log，成功生成 AXF/HEX，0 Error(s)、0 Warning(s)；日志 MDK-ARM/tmp/mode3_build.log。Windows 沙箱初始化失败，本次授权源码/文档写入及验证使用沙箱外执行路径。


## 12. 2026-10-17 MODE3 改用用户文字谱

- 基线仍为 TEST_TIGERs_Archetecture / 2245581e24d732d1969edf2b6c689e8e04b10931；保留上一任务尚未提交的 freertos.c、music.h 和本文改动。
- 按用户后续要求，Core/Inc/music.h 中原片头音符表原样保留并改名 pepa_pig；新文字谱命名 mode3_music，现有播放函数直接使用新表，无需修改播放逻辑。
- 采用 C 调，保持原音区约定 1=C5（523 Hz），6=A5（880 Hz），高八度 1'=C6（1047 Hz）；每音 1/4 拍即一个十六分音符，沿用 96 BPM。末尾 7 - - - 为同一音延长四个单位，不重复触发，非休止。
- 新谱 50 个发声音符，共 53 个十六分音符单位；现有整数毫秒播放累计 8269 ms（理论约 8.281 s，不含调度误差）。本节取代第 11 节“当前 MODE3 播放佩奇片头约 15.6 秒”的描述，旧表仅作为备用保留。
- 验证：tmp/mode3_replace.py 独立按用户文字谱逐音检查频率与时值；GCC -Wall -Wextra -Werror 编译实际播放函数模拟，50 个音符 / 8269 ms、结束静音、恢复 ARR 和关机中断均通过。Keil 完整工程增量构建通过，生成 AXF/HEX，0 Error(s)、0 Warning(s)，日志 tmp/mode3_replace_build.log。git diff --check 通过。
- 未烧录或实机试听；其他模式、后续 MPU6050 提示、硬件配置及保护逻辑沿用上一实现。


## 13. 2026-10-17 增加 MODE3 单行选曲接口

- 基线仍为 TEST_TIGERs_Archetecture / 2245581e24d732d1969edf2b6c689e8e04b10931，保留现有工作区及用户 cfg.h 修改。
- 用户要求可直接指定曲目名称，免去交换数组名。Core/Inc/music.h 新增 `#define MODE3_MUSIC pepa_pig`，改为 `mode3_music` 即选用用户文字谱；两份音符数据及名称保持不变。
- Core/Src/freertos.c 播放函数读取 MODE3_MUSIC，MODE3_MUSIC_COUNT 也由该宏的实际数组计算，当前已选佩奇。
- 本轮读取时发现 COUNT 已改用 pepa_pig，但播放器仍读取 mode3_music，两者长度不一致会越界；本次统一曲目与长度来源，消除此隐患。此前建议交换数组名称不够方便，现以本接口为准。
- tmp/music_select_edit.py 提取当前播放函数，GCC -Wall -Wextra -Werror 分别模拟两种选曲；佩奇 73 个有声音符 / 15600 ms，新谱 50 个 / 8269 ms，均验证结束静音、恢复频率和关机中断。git diff --check 通过。
- 未烧录或实机试听。
- Keil 工程构建通过，AXF/HEX 已生成，0 Error(s)、0 Warning(s)，日志 MDK-ARM/tmp/music_select_build.log。


## 14. 2026-10-17 新增 valorant 开机曲

- 当前基线：TEST_TIGERs_Archetecture / d76a6f1；开始时 music.h 已有工作区修改，保留其内容。
- 用户要求新增 `3 3 - 3 6`，命名 valorant，开机只播一遍。Core/Inc/music.h 新增 valorant[]，频率/时值为 {659,1}, {659,2}, {659,1}, {880,1}；沿用 C5 基准和 96 BPM、每单位 1/4 拍，横线延长第二个 3，合计四次发声、五单位时值。
- MODE3_MUSIC 已选择 valorant，原 pepa_pig 与 mode3_music 均保留。播放器已有 MODE3 启动单次调用，无循环重播，随后仍继续原 MPU6050 自检及其提示音。
- tmp/valorant_edit.py 提取当前播放函数并用 GCC -Wall -Wextra -Werror 编译模拟，通过 4 个音符 / 780 ms、结束静音、恢复原频率和关机中断；git diff --check 通过。未烧录、未实机试听。
- Keil 工程构建通过，生成 AXF/HEX，0 Error(s)、0 Warning(s)，日志 MDK-ARM/tmp/valorant_build.log。


## 15. 2026-10-17 修正 valorant 横线为间隔

- 基线 TEST_TIGERs_Archetecture / d76a6f1，保留现有未提交修改。
- 用户明确 `3 3 - 3 6` 中横线表示间隔，纠正第 14 节的延音解释。Core/Inc/music.h 中 valorant 改为 {659,1}, {659,1}, {0,1}, {659,1}, {880,1}，频率零表示静音；五段各 1/4 拍，96 BPM 下整数时长仍为 780 ms。
- 继续选择 valorant，MODE3 开机只播放一次；其余曲目保持现状。本次解释修正针对用户当前 valorant 谱。
- 验证：Keil 增量构建通过，生成 AXF/HEX，0 Error(s)、0 Warning(s)，日志 tmp/valorant_rest_build.log；git diff --check 通过。未烧录或实机试听。


## 16. 2026-10-18 音乐修订与开机轮动、功率板不升压分析

### 本次基线与实机反馈

- 分支 TEST_TIGERs_Archetecture，HEAD d76a6f1（Add music）。开始时 AGENT.md、Core/Inc/music.h、MDK-ARM/SRC_Firmware.uvprojx 已有改动；保留工程中的 ARMCC 5.05 工具链选择，不覆盖用户修改。
- 用户实机反馈：右前轮在刚上电、音乐之前突然动一下，其他轮子不动；使用一块 6S 电池，显示 BAT=22.2 V、CAP=0.1 V，界面没有温度显示。以上是用户观测，不是本次连接硬件测得。
- 本地 TIGERs 仍是 E:/SRC/Tigers/Firmware-master/Firmware-master 的 main2019 / README v2020 参考，未取得可靠提交号，不代表现役比赛固件。本次比较直接读取本地源码，未构建 TIGERs。

### 当前音乐接口与实际改动

- Core/Inc/music.h 中保留 pepa_pig、mode3_music、valorant 三个数组。MODE3 开机曲由单行 `#define MODE3_MUSIC valorant` 选择；MODE3_MUSIC_COUNT 自动取同一数组长度，切换无需改数组名或播放器。
- 用户最新明确谱为 `2 2 _ 2 5-`：每个短音与下划线休止占半拍，末音 5 连续保持一拍。沿用 96 BPM、C5 为 1，2=D5≈587 Hz，5=G5≈784 Hz。
- valorant 数据为 `{587,2}, {587,2}, {0,2}, {587,2}, {784,4}`；sixteenths 字段每单位为十六分音符（1/4 拍），零频率表示静音，最后一项是一整个持续音，不拆分重触发。
- Core/Src/freertos.c:398 Play_Mode3Music 继续只在 SELFTEST_MODE 的启动处调用一次，且在主运行循环之外；调整末项 gap=0，完整维持其写定时值，并跳过 osDelay(0)。其他短音仍保留原有 10% 分音间隔；末音的持续过程中无中途静音。
- 整段共三拍，理论 1875 ms；整数毫秒累计 1873 ms，其中休止 312 ms，末音连续 625 ms。播放后静音并恢复原 ARR，后续 MPU6050 检测及提示音仍执行。
- 本节取代第 14、15 节关于当前 valorant 音高、时值和符号的旧解释；另两首数组内容不变。末项完整时值规则也适用于通过宏选择的其他曲目。

### 右前轮在音乐前突动：可确定与尚未确定

后续更新：本节为取得电机驱动原理图前的分析。现已确认四轮均为 A4931，PWM 和 BRK 均有5.1 kΩ上拉；实际启动顺序修复与证据边界见第17节。

- Core/Src/main.c:104 起先初始化 GPIO、TIM1 等外设，再 SRC_Robot_Init。robot.c:53 起 init_gpio、init_motor、其他硬件与参数初始化后才启动 RTOS。
- freertos.c:279 播放音乐，完成后还有 MPU6050 初始化和提示音；freertos.c:327 才调用 start_motor，然后启动 TIM12 并置 startup_complete。因此音乐之前不存在这条路径的正常轮速 PID/PWM 启动或自检旋转命令。motor.c:135 在 is_motor_run=0 时直接返回，开机目标与 PID 也初始化为零。
- MODE3 的确在 do_robot_run 中执行正反转自检，但它位于以上启动流程之后，且给四轮的车体旋转目标，不能解释用户所述“音乐之前只动一轮”。
- 编号必须区分逻辑轮与板上丝印：robot.c:81 的几何顺序为左前、右前、右后、左后。右前轮是 g_robot.wheels[1]，motor.c 将它映射至 TIM1_CH2 / PE11（宏名 MOTOR_PWM4），DIR4、BRK4、编码器 TIM3。故用户说的板上 MOTOR4 可与右前对应；它不是 wheels[3]。逻辑第4轮是 wheels[3]、TIM1_CH4 / PE14（MOTOR_PWM2）、左后轮，须以实物接线核验。
- 重点嫌疑是 PWM 尚未使能时的电平，而非稳态占空比：tim.c:78 采用 PWM1/低有效极性、Pulse=0；空闲 OIS=RESET，OSSI/OSSR 禁用；tim.c:848 将四个 PWM 脚设为 AF_PP + NOPULL。CCR=0 在 PWM 正常输出时表示高电平关闭，不能据此推断在复位/复用切换/MOE 未开启阶段引脚也一直为高。
- gpio.c:59 首先将 BRK/DIR 写低，init_motor 后又将四路 BRK 写高，而此时 TIM1 的正常 PWM 还未启动。如果实际轮电机驱动是低有效 ENABLE、低有效 BRAKEZ 且未反相，BRK 拉高会释放刹车，PWM 若处于低/未被可靠上拉，就可能短暂通电。A4931 数据手册确为 ENB低=驱动、BRAKEZ低=刹车；但本次未获得四路轮驱动原理图/芯片确认，不能把 MOTOR5 的 A4931 接法直接当成四轮的既定接法。
- 四通道配置相同，软件证据尚不能解释为何只有右前发生；通道特有的上拉/下拉、供电先后、驱动输入、接线和电机启动响应均须查实。音乐前发生也使 start_motor 内依次开启四通道的顺序不是首要解释。
- 实测建议：对右前 PE11 / PE5（BRK4）与一个正常轮的 PWM/BRK 同时抓取从上电到音乐前的波形，核对驱动端的使能极性。若发现有效脉冲，应采用“先可靠关闭 PWM/保持刹车，再初始化并建立零输出，最后释放刹车”的顺序，必要时核对外部偏置电阻；本次分析不擅自改变四轮行为或引脚配置。

### kicking board 使能：SRC 与 TIGERs 逐项对比

| 项目 | 当前 SRC | TIGERs 本地 main2019 参考 |
| --- | --- | --- |
| 最终充电输出 | PC4 / CHG_EN，高=允许，低=关闭；board_monitor.c:419 | PG11 / chargePin，高=允许；shared/module/kicker.c:412 |
| 上层使能 | 没有独立 AUTO_CHARGE 请求，BoardUpdate 10 ms 周期运行，条件满足即充电，MODE3 无特殊禁充 | 必须 KICKER_CHG_MODE_AUTO_CHARGE；比赛链由 enabledSystems 与包中 KICKER_AUTOCHG 决定，robot/robot.c:602；测试显式调用 KickerSetChargeMode，main2019/test/test_kicker.c:78 |
| 板存在检测 | 没有同样的 CHG 弱上下拉识别；直接将 CHG_EN 配为低电平输出 | identifyKickerBoard 对 CHG 弱下拉/上拉检查板载偏置，通过后配输出；不是额外独立使能线，不能脱离硬件照搬检测条件 |
| ADC | SPI2，CPOL高、第二沿、8bit、MSB先、APB1/8≈5.25MHz；两组各16样本，舍前2个，至少4个合法样本；零温度通道或温度超 -20..150°C 判无效 | 相同 CPOL/CPHA、分通道/舍样和换算方法；独立任务约1ms发布，采样事件超过10ms阻断充电；SPI分频不可只按数值比较，要结合其H7总线时钟 |
| 电容目标 | 180 V，低于175 V开启，到180 V关闭 | 默认配置200 V、回差5 V，可由配置文件覆盖；踢球测试临时220 V |
| 电源条件 | 额外要求 INA226 有效且电压>3.3×BATTERY_CELLS；6S为19.8 V；缺数/低压持续5s会锁存 quiet，数据恢复后不会自动解除 | chargeControl 不直接要求 INA226 标志或相同6S阈值；电源/机器人系统另有管理，不能理解为整机不检查电池 |
| 温度 | >=60°C停充，<=55°C恢复；有效范围检查比 TIGERs ADC 驱动更严格 | >60°C停充、<55°C恢复 |
| 进度监测 | 充电3s未增加5V，暂停30s重试 | 同类3s/5V/30s看门狗 |
| DONE/FAULT | 额外读取 PC13 / KICK_ERR，低有效；充电中低电容时低电平触发 charge_fault | 此份 KickerData/chargeControl 不包含独立 DONE/FAULT 输入；主要通过 ADC 电压和进度判断 |
| 踢球期间 | TIM9 CEN=1 时停充 | KICKING 及 COOLDOWN 均停充；SRC没有对应整段 cooldown 的同一门控 |

- 依据：SRC Core/Src/board_monitor.c:332、Core/Src/spi.c:41、Core/Inc/cfg.h:248；TIGERs src/shared/module/kicker.c:162、:211、:322、:477，src/shared/drv/adc_kicker.c，src/main2019/tiger_bot.c:137，src/main2019/robot_impl.c:452，以及测试/机器人调用链。
- LT3751 的 CHARGE 高于1.5 V启动、低于0.3 V停止，与两份源码“拉高允许充电”一致；DONE 低可以表示目标到达，也可以由 FAULT 引发，不能只凭名称认作“充好了”。依据官方 [LT3751 数据手册](https://www.analog.com/media/en/technical-documentation/data-sheets/lt3751.pdf) 第7–8页引脚定义及第20页 DONE/FAULT 说明。若当前板通过反相/隔离接入，仍需实测控制器端。
- 当前 KICKER_CALIBRATION_CONFIRMED=1、BATTERY_CELLS=6。不能再把“标定宏没开”或“自检缺少 TIGERs AUTO_CHARGE 指令”当成本次不升压的既定原因；SRC 当前路径本就自动充电。
- 用户22.2 V高于19.8 V阈值。OLED 从 g_robot.bat_v_f / cap_v_f 显示，它们在当次板更新中仅于对应 valid=1 时取实测值，否则清零；因此持续非零显示支持“有过有效测量”，但不是 quiet、charge_fault 或 hot 的显示。0.1 V也只代表近零且有约0.061 V/LSB量化，不能据此判定充电线或功率级正常。
- OLED正常页确实没有温度/充电禁止原因一栏（misc.c:126），不是温度传感器必然没采到。有效 ADC 温度仍可能>=60°C而禁止充电。
- 现有代码在有效22.2 V、温度25°C、电容接近0.1 V、无故障且未quiet/shutdown时应该拉高 CHG_EN；已用实际 board_monitor.c 的宿主模拟复现。未实测PC4/PC13和状态快照，无法确定此次实机究竟是软件门控、接口连线还是LT3751功率级原因。

### 实机定位顺序（不绕过保护）

1. 观察 PC4 / CHG_EN 在开机后及至少一个30s重试周期内的波形，并核对功率板 LT3751 CHARGE 端是否收到对应高电平。仅万用表可能漏掉短暂开启。
2. 若 PC4 始终低或很快拉低，读 board_status（或 board_monitor_get 快照）：kicker_valid、battery_valid、temperature、charge_enabled、charge_fault、kick_done、quiet、shutdown，并看 TIM9 CR1.CEN。正常应为 valid均1、温度<60且未锁热、fault/quiet/shutdown均0、CEN=0。
3. 若低电容时 kick_done=1，检查 PC13 的 KICK_ERR 实际连到 DONE 还是 FAULT、是否低电平、上拉及其电源。此输入会使本实现迅速进入30s禁充，而 TIGERs 本地路径没有同一输入条件。
4. 若 quiet=1，即使此时BAT显示22.2 V仍禁充，检查启动早期 INA226 是否曾无效/低压持续5s；quiet当前是锁存保护，本次未修改。
5. 若 PC4高但控制器CHARGE端低，查连接/反相/接口；若两端均高但电压不升，查实际 VCC、VTRANS、FAULT 与升压功率级。LT3751 的硬件 UVLO/OVLO可独立阻止开关，MCU高电平不是“升压电路正常”的证据。硬件/测量记录补全后再决定修改，而非强行拉高使能。

### 本次验证与边界

- 新增 MDK-ARM/tests/test_startup_music.py，提取当前生产播放函数，GCC -Wall -Wextra -Werror 模拟验证实际发声起点、2/5音高、312ms休止、625ms完整末音、1873ms总时长、关机中断、结束静音及原频率恢复，均通过。
- python tests/test_board_migration.py：4S/6S原有 power/charge/pulse/FG/关机/放电/故障/超时/quiet/缺数用例全部通过。
- tmp/observed_boot_charge.py 复用生产板级测试桩，以6S、22.2 V、模拟0.1 V电容（量化为0.061 V）、约25°C验证CHG_EN会高；再注入过温及低电平KICK_ERR，确认相应禁止条件。此为宿主可控条件验证，不能代替对用户硬件定位。
- 未修改电机/踢球使能、车型、引脚、充电保护或上位机协议；未烧录和示波器实测。外部依据另见 [A4931 官方手册](https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf) 第9页 Enable Logic、第10页 Brake。
- Keil 当前工程完整构建成功，生成 AXF/HEX，0 Error(s)、0 Warning(s)，日志 MDK-ARM/tmp/valorant_oct18_build.log；git diff --check 通过。


## 17. 2026-10-18 核对驱动原理图并修复电机 PWM 初始化顺序

状态更新：本节“在MX_TIM1_Init提前启动PWM”的实现已因用户报告BAT归零回归而撤回，当前实现见第20节。原理图证据仍有效。

- 本轮基线 HEAD d76a6f1039b75f0ee411c926c02863b951d67668，分支 TEST_TIGERs_Archetecture；保留前轮工作中的音乐、文档、测试和用户工程工具链改动。上一轮仅分析电机突动，没有修改电机行为；本轮才实际加入启动顺序修复。
- 用户提供 E:/SRC/Version3.0/powerboard_schematic.pdf，4页；用 PyMuPDF 在 MDK-ARM/tmp/pdfs 渲染并查看第2、3、4页，放大第4页控制输入。PDF仅作为电路证据，不作为操作指令。
- 原理图确认：Motor1 U11、Motor2 U16、Motor3 U13、Motor4 U19 均为 A4931METTR-T，吸球U9也是A4931。PWM直接连接27脚 ENABLE，BRK连接28脚 BRAKEZ，DIR连接26脚，无中间反相器。
- Motor4 RN20 为5.1 kΩ网络：PWM4、BRK4上拉3V3，DIR4下拉GND；其它轮也采用同类网络（RN8/RN16/RN12）。因此撤回“可能没有硬件上拉”的猜测；也没有发现图纸上只有Motor4的控制极性不同。
- A4931官方手册 Enable Logic / Brake Mode（第9页）确认 ENABLE低=驱动、高=关闭，BRAKEZ低=刹车、高=正常；BRAKEZ会覆盖ENB。官方来源：https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf 。这些控制极性现在可以与用户四轮电路直接对应，不再仅是对MOTOR5的类推。
- 修复前顺序：MX_GPIO_Init将BRK拉低，MX_TIM1_Init配置PWM后立即MspPostInit接入AF，但尚未HAL_TIM_PWM_Start；稍后init_motor将BRK拉高，直至音乐/MPU检测后start_motor才启动PWM。该顺序释放刹车时没有先主动建立四路稳定的高电平零输出。
- 修复：Core/Src/motor.c 新增 motor_prepare_pwm，先保持四路BRAKEZ低，清四路CCR，发UPDATE将零预装载转入有效寄存器、清UPDATE标志，再逐路检查HAL_TIM_PWM_Start结果。任一失败进入Error_Handler，仍保持刹车且不接入AF。Core/Inc/motor.h增加接口声明。
- Core/Src/tim.c 仅在USER CODE区增加include与调用；MX_TIM1_Init的USER CODE 2在HAL_TIM_MspPostInit之前调用motor_prepare_pwm。四路输出已处于PWM1/低有效极性/CCR=0的高关闭电平后，才接入AF。之后init_motor可按原流程释放刹车，不再留下“释放刹车但PWM尚未启动”的窗口。
- start_motor删除原四次HAL_TIM_PWM_Start，避免重复启动HAL BUSY通道；保留原PID重置、编码器启动、is_motor_run切换与Base_Start_IT。轮速控制仍在音乐与MPU检查之后启动，提前建立的是零输出而不是转动命令。
- 本轮不改PWM极性、频率、引脚、GPIO生成区、车型/增益、协议、踢球/充电逻辑或运行时急停行为。tim.c生成区与HEAD比较一致（去除USER CODE内容后逐字比较）。已有板载上拉负责MCU复位至GPIO接管前的关闭电平，本修复针对固件可控制的初始化阶段。
- 新增MDK-ARM/tests/test_motor_startup.py，提取实际生产函数，GCC -Wall -Wextra -Werror模拟：非零预装载清零并UPDATE、在四路高电平输出就绪前不释放刹车、IRQ mask保持、PID延后且不重复启动HAL通道、禁用时不启动，以及UPDATE/四通道启动的5种失败路径。全部通过。
- python tests/test_board_migration.py：原4S/6S保护/充放电/输出用例全部通过；python tests/test_startup_music.py也通过。
- 边界：已修复代码中的启动时序缺口，尚未烧录或测量真实上电波形，不能宣称已实测消除突动。四路电路同类，此修复也不能证明“只动右前”的唯一物理原因；若烧录后仍发生，应同步测Motor4的PE11/PWM4、PE5/BRK4及3V3与正常轮对照，核对RN20装配、连接器接触及实际电源时序。
- Keil完整工程构建通过，生成AXF/HEX，0 Error(s)、0 Warning(s)，日志MDK-ARM/tmp/motor_startup_build.log；git diff --check通过。


## 18. 2026-10-18 用户反馈电池显示突然为零：调查中

- 基线仍为 d76a6f1 工作树；用户在电机启动顺序修改后反馈“电池电压显示0，刚才正常”。尚未取得断电重启结果、CAP值及板级状态，不能认定硬件损坏或确认由启动修改导致。
- board_monitor.c:432 在battery_valid=0时明确将g_robot.bat_v_f置0。INA226初始化失败、I2C事务失败或超过100ms没有新的转换结果，均可能走到无效/归零；另一个可能是有效事务本身返回bus=0，需区分而非直接把显示零等同于无效。
- 核对git diff：本轮之前的电机修复未改board_monitor.c、i2c.c、gpio.c或main.h。INA226仍用I2C3、PA8/SCL、PC9/SDA、100kHz、7bit地址0x40。提前调用的是非中断版HAL_TIM_PWM_Start，正常轮速更新仍由后续TIM12启动；未发现源码上的I2C引脚复用冲突。但这不能排除实机时序、电源或总线恢复问题。
- 优先诊断board_status.ina226_ready、battery_valid、battery_tick、bat_v、quiet，并配合hi2c3.ErrorCode及I2C3->SR2.BUSY。ready=0先查初始化/通信，ready=1且valid=0查转换完成和读取超时；valid=1而bat_v=0再查原始总线电压寄存器/硬件测点。
- 当前battery_valid=0会禁止充电；无有效数据持续5s会锁存quiet，即使之后显示恢复也可能仍禁止执行器。未用保留旧电压数值或取消保护掩盖错误。
- 复跑python tests/test_board_migration.py，4S/6S全部用例通过，包括I2C失败/恢复、超时及quiet；仅为宿主模拟，未连接用户机器。本轮只记录调查，未新增固件行为修改。
- 用户随后确认：烧录电机启动修复后才出现BAT=0，断开电池及USB后重启仍为0；CAP未连接；原先的轮子突动已消失。后者为用户实机验证结果，更新第17节尚未取得实机结果的状态。BAT问题仍需INA226状态定位，不能归因于CAP未连接：当前流程不以kicker_valid为条件才读取电池。


## 19. 2026-10-18 根据实测0x220增加I2C3异常恢复（已撤回）

状态更新：自动复位只能恢复故障，不能解释回归原因。该实现和对应调试全局已在第20节撤回；以下保留为历史工作记录，不是当前固件行为。

- 用户截图：hi2c3.ErrorCode=0x00000220；board_status.ina226_ready / battery_valid 显示cannot evaluate，并未取得这两项的数值。后者是调试符号/表达式求值问题，不能当作传感器状态或数值零。
- 本地HAL精确解码：stm32f4xx_hal_i2c.h:169 的HAL_I2C_ERROR_TIMEOUT=0x20，:172的HAL_I2C_WRONG_START=0x200（注意该宏名不含ERROR）。I2C_RequestMemoryRead/Write在等待SB超时且CR1.START仍为1时置WRONG_START，之后BUSY等等待可再累积TIMEOUT。因此已取得I2C传输启动异常/超时证据，不是已经读到真实零电压；尚未证实具体物理触发原因或与提前启动PWM的因果关系。
- 官方依据：STM32F405/407 ES0182 Rev19 §2.10.3说明异常STOP可能导致无法生成START，建议SWRST复位。来源 https://www.st.com/resource/en/errata_sheet/es0182-stm32f405407xx-and-stm32f415417xx-device-errata-stmicroelectronics.pdf 。本次以该建议补充恢复，不声称用户硬件已被证明触发这一特定勘误场景。
- Core/Src/i2c.c的USER CODE区新增I2C3_RecoverIfNeeded：仅在WRONG_START/TIMEOUT/BERR/ARLO或BUSY时，关闭I2C3、置/清SWRST、HAL_DeInit/Init重建外设与HAL状态，返回是否成功。正常通信和仅NACK不做复位；不修改总线频率、地址、引脚或I2C2。
- BoardUpdate独占I2C3，恢复接在board_monitor.c原有INA226每秒初始化重试入口，最多随每秒重试调用，不在ISR中执行；复位后仍必须重新验证ID、配置并读到完整新样本才能battery_valid=1。未取消低压、quiet或充电保护，也未用旧电压伪造有效读数。
- 原先只有状态寄存器读取失败会退出ina226_ready；现将分流/电压寄存器读取失败也设ready/valid=0，避免数据阶段START超时后一直留在原读循环、绕过初始化恢复入口。失败当次立即禁止使用该样本。
- 新增可Watch的volatile全局g_ina226_last_i2c_error（复位前错误）和g_ina226_recovery_count（恢复尝试次数），声明位于Core/Inc/i2c.h。当前hi2c3.ErrorCode会被HAL重置，判断历史故障应结合last与计数，不能以last仍0x220推断恢复失败。
- tests/test_i2c3_recovery.py提取实际恢复函数，GCC -Wall -Wextra -Werror验证0x220、BUSY、软件复位顺序、保留证据、健康/NACK不复位、DeInit/Init失败返回，全部通过。
- 扩展tests/test_board_migration.py：分别注入状态、分流、电压寄存器失败以及恢复失败，验证失效停充、每秒重试、重新采样后恢复；4S/6S全部既有及新用例通过。test_motor_startup.py通过。
- 运行音乐测试时发现用户已将当前valorant时值改成1/1/1/1/2，保留该工作区修改不覆盖。旧测试固定断言2/2/2/2/4因此失败；已将播放器测试改为独立参考曲谱以验证休止和末音连续性，不再绑死用户选曲/节奏；更新后测试通过。当前music.h注释仍是用户改时值前的描述，本次电池修复不修改曲谱。
- i2c.c生成区与HEAD对比一致。待烧录验证BAT是否恢复；若恢复计数持续增加而仍读不到，应测PA8/SCL、PC9/SDA电平及波形、INA226供电与连接。软件复位不能释放被外部持续拉低的信号线，也不能证明线路完好。
- Keil完整工程构建成功，生成AXF/HEX，0 Error(s)、0 Warning(s)，日志MDK-ARM/tmp/i2c3_recovery_build.log；git diff --check通过。本次未烧录，恢复效果待用户实机验证。


## 20. 2026-10-18 回归复盘：撤回过早PWM与I2C自动复位，改用启动期保持刹车（历史方案，由第24节替代）

- 用户质疑“故障后重启没有根本解决，为什么改代码前不会”。认可该质疑：第19节的SWRST是恢复手段，尚无证据证明故障触发机制；不能把错误码解码、勘误建议或宿主测试当成根因已确认。
- 用户提供的回归时间关系：第17节过早启PWM版本在实机消除了轮动，但BAT原本正常、烧录后持续为0且断电不恢复。最应先隔离我引入的启动顺序变化；尚未取得同硬件A/B结果，不能声称PWM与I2C存在已证实引脚冲突或必然电气联系。
- 第17节使TIM1在MX_TIM1_Init中启动，位于I2C3等外设初始化之前；这比仅调整刹车释放所需的范围更大。本轮撤回该调用与tim.c新增include，tim.c与HEAD逐字无差异。
- 当前最小方案：SRC_Robot_Init用motor_hold_startup替代启动处的init_motor，保持四路BRAKEZ低、DIR与CCR清零、is_motor_run=0，不启动PWM。其它运行时init_motor（包括保护路径）语义不变。
- start_motor仍在音乐/MPU检查之后运行，与原基线的PWM启用时机一致：重置PID/编码器后，在短暂、仅寄存器操作的PRIMASK临界区中调用motor_prepare_pwm，清零并UPDATE预装载、检查启动四路PWM成功，再次核对禁用状态，然后BRAKEZ拉高并is_motor_run=1。任何启动错误仍保持刹车并进入原Error_Handler。临界区保存恢复原PRIMASK，防止高优先级关机任务在输出建立过程中提前释放刹车。
- 不再要求“PWM必须先于GPIO复用接入”。GPIO复用可以按原顺序发生，电机在BRAKEZ低时被硬件刹车覆盖；关键不变量是“在全部零输出就绪之前不释放刹车”。这避免为防突动去改变其余外设初始化期间的TIM1运行状态。
- 撤回第19节的I2C3_RecoverIfNeeded、两个调试全局及数据读取分支调整。Core/Src/i2c.c、Core/Inc/i2c.h、Core/Src/board_monitor.c与HEAD逐字一致，保留原无效数据/电源保护，不以自动复位掩盖验证结果。原恢复专项测试移到忽略的tmp/test_i2c3_recovery_withdrawn.py保留历史，板级测试也恢复HEAD。
- tests/test_motor_startup.py按当前实现更新：验证外设/音乐/IMU阶段只有保持刹车、没有PWM启动；原时机建立全零有效比较值后才释放刹车；PRIMASK保持；禁用路径和UPDATE/各通道失败均不放行。测试通过。原4S/6S板级回归和音乐参考谱测试通过。
- 保留用户对music.h时值的当前修改及工程工具链选择。本次不改I2C频率/地址/引脚、充电条件或运行时控制参数。
- 当前版本是缩小范围的回归验证与启动顺序修正，不宣称已经实测修复BAT。需用户烧录后确认BAT恢复且不突动；若恢复，支持过早PWM启动这一变更与回归相关，但具体物理触发机制仍需波形/寄存器证据。若未恢复，继续以0x220为线索定位，不继续叠加未经验证的恢复补丁。
- Keil当前工程构建成功，AXF/HEX已生成，0 Error(s)、0 Warning(s)，日志MDK-ARM/tmp/motor_startup_minimal_build.log；git diff --check通过。本次未烧录。


## 21. 2026-10-18 Valorant 升高一个八度

- 基线仍为 TEST_TIGERs_Archetecture / d76a6f1 工作树，保留已有未提交改动。
- 按用户要求仅将 Core/Inc/music.h 中 valorant 的 D5≈587 Hz 改为 D6≈1175 Hz，G5≈784 Hz 改为 G6≈1568 Hz；静音仍0 Hz。三个D音和最后G音全部升高一八度。
- 保留用户当前时值1/1/1/1/2、96 BPM、MODE3选曲和开机单次播放。同步更正原曲谱注释，避免仍声称当前数组使用2/2/2/2/4。电机、电池与其它两首音乐均未改动。
- Keil构建通过，生成AXF/HEX，0 Error(s)、0 Warning(s)，日志MDK-ARM/tmp/valorant_octave_build.log；git diff --check通过。未烧录或实机试听。


## 22. 2026-10-18 音符对照表与BAT再次归零的无恢复诊断

- 用户要求将每个音符对照表写入music.h，并反馈BAT又显示0。当前HEAD仍d76a6f1，保留此前所有工作区改动和用户当前valorant时值。尚未取得“最小启动版本是否曾恢复BAT”和本轮ErrorCode新读数，不能把“又归零”直接当成已证明的同一故障或升八度导致。
- Core/Inc/music.h新增注释对照表，按A4=440 Hz十二平均律计算并取整，列出1/#1/2/#2/3/4/#4/5/#5/6/#6/7对应C/C#/D/D#/E/F/F#/G/G#/A/A#/B，覆盖C4–B7；明确本工程1=C5、1'=C6，及0/_休止、八度、sixteenths单位和时值对照。未修改任何歌曲数组、BPM、选曲或播放逻辑。
- 电池显示仍由battery_valid决定，无效时写0；有效读数本身为零也可显示0。上一轮升八度只变更TIM8音符频率，不改I2C3的PA8/PC9、100kHz、0x40地址或调度。审查set_buzzer保存/恢复PRIMASK与播放器ARR更新路径，尚未找到音高数据直接修改I2C3的调用或共享引脚；这不是排除实机噪声、供电或时序问题的证据。
- 继续遵守第20节，不增加自动复位、软件清BUSY、总线脉冲或新的启动顺序改动。用户上次0x220仅说明HAL的START异常和超时，本轮是否相同仍需读取。
- 为避免static board_status在Keil中cannot evaluate而无法定位，新增公开volatile结构g_battery_diag，类型/声明在Core/Inc/board_monitor.h，记录代码在board_monitor.c。它是调试观察对象，不是无线协议字段或控制输入，不改变board_status_t布局。
- 首次I2C事务失败时记录first_failure_ms、first_error、first_hal_status、first_register、first_write，以及当时scl_high/sda_high（只读GPIO IDR，1=高）。后续重试只增加failures，不覆盖首次现场；上电/board_monitor_init时清零。未读取具有清标志副作用的SR1/SR2，也不清除hi2c3.ErrorCode。
- 同时发布最近成功读到的manufacturer_id、device_id，当前ina226_ready、battery_valid、quiet、bat_v与battery_age_ms。bat_v可能是历史有效值，判断现值必须结合battery_valid；首次错误仍保留也不等于当前还在故障。
- 诊断解释：first_register=0xFE/0xFF定位ID阶段，0x00定位配置，0x06定位Mask/Enable，0x01/0x02定位分流/电压；first_write区分读写。failures=0而ready=0时应看ID是否匹配5449/226x，而不能仅看I2C错误码。首次SCL/SDA采样为低不能直接区分MCU外设或外部器件拉低，仍需波形。
- tests/test_board_migration.py增加读/写失败诊断用例，验证首错不被后续错误码和GPIO变化覆盖、恢复后当前状态更新、缺失CAP不妨碍电池读取；4S/6S新旧用例全部通过。test_motor_startup.py及test_startup_music.py通过。正常采样判断、超时、quiet、充电/关机条件均保持原样。
- 已请求用户说明最小启动版本BAT是否曾恢复以及本轮ErrorCode。下一步烧录后在Watch展开g_battery_diag并截图，以真实首次失败阶段和线电平继续定位，而非再按猜测叠加恢复补丁。没有实机证据前不宣称BAT根因已解决。

- 第22节构建结果：Keil生成AXF/HEX，0 Error(s)、0 Warning(s)，日志tmp/music_table_battery_diag_build.log；git diff --check通过。未烧录。

## 23. 2026-10-18 用户确认Q1损坏、测试电机支路保险丝熔断

### 新实机事实与此前结论修正

- 用户先确认：撤回提前PWM的版本BAT曾恢复；后来又出现归零，但随后已能读电池。同时出现只接电池、无USB/调试器时插电即上电，OLED/音乐及全部功率板灯均启动，按键不能关机。
- 用户进一步自行找到Q1损坏，且插测试电机的对应支路保险丝熔断，怀疑电机启动瞬间大电流。本次未在实物上测到失效模式、熔断时刻或电流波形，因此将“Q1损坏、支路熔断”作为用户确认事实，将“电机启动浪涌烧毁Q1”保留为待验证假设。
- 若Q1漏源击穿短路，会旁路受控断电路径，可解释电源控制器发出关机也无法断开负载。不能把这一现象归为新增音乐或电池诊断代码的自动开机指令；当前固件没有此类指令。Q1损坏也不能单独证明全部BAT异常均由硬件造成。
- 原理图第1页：LTC2955-1的ON接地、BTN接PB#、EN接LTC4231 SHDN#；KILL为MCU开漏关断请求。已按ADI LTC2955手册核对ON为自动上电输入、PB低有效，ON不用时接地。这解释了此前应区分硬件上电与固件关机流程的原因，不以未测信号代替故障确认。

### 必须优先核对6S与TVS，而非直接认定电机浪涌

- 提供的powerboard_schematic.pdf第2–4页将各电机支路TVS标为SMAJ18A（四轮D6/D19/D13/D25，吸球D1），在相应支路保险丝之后并联在电源与GND之间。
- Vishay官方SMAJ系列手册88390第2页：SMAJ18A的VWM=18 V，VBR=20.0–22.1 V（IT=1 mA），VC=29.2 V是在规定峰值脉冲电流下的箝位值，不能把29.2 V当持续工作上限。
- 用户6S标称22.2 V，按4.2 V/节满电为25.2 V。若实装确为SMAJ18A，则已超过其持续反向工作电压，甚至高于所列击穿范围上限；存在持续雪崩发热、失效及支路过流的风险。尚未核实实物丝印/品牌，不能断言TVS就是本次熔断唯一原因。
- 即使拆下电机，TVS仍连接在保险丝后方；故“某电机支路保险丝熔断”不等于“电机绕组启动电流已经测得过大”。应先断电检查该支路TVS和功率桥MOSFET是否短路，再用限流电源空载定位。不要仅更换Q1/更大保险丝后直接重复接6S。
- 这是前次收到原理图时漏查的电池/器件电压兼容性，应优先补查；软件BATTERY_CELLS=6不能证明硬件已适配6S。此前关于沿用硬件的描述不构成6S兼容验证。
- 来源：https://www.vishay.com/doc?88390= 。未在本轮指定TVS替代型号，因为需同时核对最大电池电压、浪涌箝位、A4931/桥MOSFET绝对额定值和回馈能量，不能只升高TVS标称电压。

### 总线热插拔与驱动保护的实际边界

- 第1页R7在LTC4231 IN/SENSE的采样位置，标为0Ω；若实装相同，相当于短接检测位置。ADI LTC4231 RevB手册第7页SENSE说明：接至IN会禁用电路断路比较器与模拟限流放大器；其正常门槛为50mV/80mV。不能假设当前板总电流有有效的主动硬限流。
- 图中仍有GATE侧R8=1kΩ、C4=10nF的电容缓升网络。它能影响输出上升速度及电容充电浪涌，但不等于建立了对短路/持续重载的电流闭环。LTC4231手册指出初始电流通常由负载电容充电主导，因此“上电浪涌”也应区分母线电容充电与电机绕组被驱动两种来源。
- Q1为FDD86367。onsemi手册的80 V/100 A等标题额定值不能直接作为热插拔线性工作能力；需按启动期间VDS、ID、脉冲时间和温度检查SOA。损坏Q1还应核查U3栅极驱动与栅极网络是否受牵连。
- A4931官方手册第9页：ITRIP≈200mV/RSENSE。图中轮驱动27mΩ对应约7.4 A的驱动采样峰值；不是整车电池电流的7.4 A精确硬上限，也不能直接与图中0466004保险丝的4 A标称值等同。实物保险丝额定值/快断曲线尚待确认。
- 同页说明BRAKEZ低时三路低侧导通，刹车循环电流不受同一采样限流约束；但静止健康电机、上桥臂关闭时，保持刹车本身不应主动向电机施加电池电压。不能仅凭本次启动期拉低BRAKEZ便认定它造成电池侧巨大电流。
- 官方依据：LTC4231 https://www.analog.com/media/en/technical-documentation/data-sheets/ltc4231.pdf ；FDD86367 https://www.onsemi.com/download/data-sheet/pdf/fdd86367-d.pdf ；A4931 https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf 。

### 固件自检仍需结合熔断时刻审查

- 当前motor_hold_startup保持刹车且不启PWM；start_motor清零/UPDATE并启动四路PWM后才释放刹车，开机没有有意给非零轮速命令。宿主顺序测试不替代上电波形。
- MODE3并非只播放音乐：robot.c:281后仍有自动吸球和正反转自检。音乐/IMU后先LED等待2s，吸球2s，平射等待2s，挑射等待2s，然后下发旋转目标±100。如果熔断发生在这之后，应查实际自检负载，不能把它统称为上电瞬间。
- do_update_motor的方向保护要求反馈速度超过门槛并且方向异常；编码器断开、反馈一直为0不会触发这一方向保护。若测试电机未接正确编码器而进入自检，PID误差可累积并持续给输出；这是需核对的风险，不是本次已证明原因。
- 已询问熔断发生在插电/按键瞬间还是自检转动后、霍尔/编码器是否连接、实物TVS型号与保险丝额定值。本轮不擅自改变电机参数、开关极性、保险丝或控制行为。
- 本轮最终增量构建（音符表、只读诊断及注释整理）成功，AXF/HEX生成，0 Error(s)、0 Warning(s)，日志tmp/music_table_battery_diag_final_build.log。硬件失效分析只更新文档，未新增控制行为或修改硬件选型。

## 24. 2026-10-18 四轮开机改为零PWM滑行，不主动刹车

- 基线：TEST_TIGERs_Archetecture / d76a6f1039b75f0ee411c926c02863b951d67668 工作树，保留其它未提交改动。用户明确要求开机不要急刹，改为PWM输出0；本节替代第20节及第23节所述“启动期BRAKEZ低”方案。
- Core/Src/gpio.c将四轮BRAKEZ（PE4/5/6/7）默认输出改为高，SRC_Firmware.ioc同步PinState=GPIO_PIN_SET。motor_hold_startup与motor_prepare_pwm同样保持BRAKEZ高、CCR=0、启动完成前is_motor_run=0。
- PWM/ENABLE低有效：逻辑零占空比对应引脚高电平。MX_TIM1_Init先将PE9/11/13/14设为推挽GPIO高；GPIO配置前先写ODR高，复位至GPIO接管前依赖图纸上的外部5.1kΩ上拉。保持原I2C及其它外设初始化顺序，没有提前启动TIM1。
- MX_TIM1_Init末尾的HAL_TIM_MspPostInit(&htim1)移到motor_prepare_pwm末尾。音乐/IMU完成后，先四路CCR清零、UPDATE装载，检查四路HAL_TIM_PWM_Start成功，最后才切GPIO至原TIM1 AF。在此之前引脚一直GPIO高；UPDATE或任一路启动失败时不会切AF，维持关闭且不主动刹车。保留PRIMASK及禁用执行器检查。
- CubeMX注意：BRAKEZ默认高已写入.ioc，但GPIO暂存/延迟AF属于运行时策略；重新生成会恢复MX_TIM1_Init末尾自动生成的HAL_TIM_MspPostInit调用，必须移除该调用并保留USER CODE中的GPIO高初始化，再运行启动测试。测试会检查该调用不能重新出现在MX_TIM1_Init中。
- 范围为四轮启动；运行时PID、故障/停机接口、吸球TIM11/BRK5及MODE3后续自动运动自检沿用现状。不以本次修改声称已消除全部过流来源。静止健康电机上的刹车不必然造成电池大电流，Q1/保险丝根因仍未实测确认。
- 用户本轮补充：无法确定保险丝熔断具体时刻；确认实物TVS为SMAJ18A，已经拆下。第23节实物型号未知这一点据此更新；6S与该型号电压不匹配得到实物型号支持，但拆除动作不是根因确证或整板6S兼容性验证。
- tests/test_motor_startup.py提取实际GPIO暂存、AF配置及电机启动函数，验证BRAKEZ不主动拉低、引脚GPIO高、零预装载和四路启动完成后才接AF、IRQ保持、禁用路径及UPDATE/四通道失败均不接AF。该测试、4S/6S板级回归、音乐参考谱测试全部通过。
- Keil构建通过，AXF/HEX已生成，0 Error(s)、0 Warning(s)，日志MDK-ARM/tmp/motor_startup_coast_build.log；git diff --check通过。未烧录，尚无实机上电电平/电流波形，宿主测试不能证明硬件故障已修复。

## 25. 2026-10-18 拆除18V TVS后仍仅U14支路熔断、Q1重复D-S短路

- 源码基线仍为d76a6f1039b75f0ee411c926c02863b951d67668工作树；本轮只分析原理图、数据手册与现有代码并更新记录，不修改固件行为、不烧录。
- 用户新实测：已拆除所有18V TVS，仍发生Q1击穿及U14保险丝熔断；只有电机接在U14对应接口才发生，接其它接口不发生；替换Q1后能恢复按键开机控制，但再次上电Q1即损坏，用户确认现为D-S短路；U14对应接口物理位置最靠近电池接口。
- 用户确认对照测试使用相同电机和线束；回复“是的；未烧录”，明确尚未烧录第24节零PWM/不刹车版本。因此不能将本轮硬件损坏当作第24节固件的实测失败或已排除软件，也不能推定当前实装旧固件的精确版本。
- 本轮用户称SMBJ18A，前述记录/图纸称SMAJ18A；两者封装命名不应混为已确认同一型号。本次核心新增证据是用户报告全部相关18V TVS已拆，故先前TVS过压假说不能单独解释拆除后的持续故障。不能据此断言原TVS此前未造成桥臂或其它器件损伤。
- 原理图第3页明确U14=Motor3 Driver支路保险丝，供电路径为电池→R7→Q1→R9→VCC→U14→Motor3桥供电。驱动U13=A4931；Q10/Q11/Q12=AON6884双N MOS，对应A/B/C三相，C47–C50为该路并联供电电容，原TVS为D13；R34=0Ω连接低侧回路与采样位置，R35=27mΩ为电流采样。不能把R34误当成第1页的总线采样R7。
- Motor3接口CN4：1=GND，2=5V_BAT，3=HA3，4=HB3，5=HC3，6=C_OUT3，7=B_OUT3，8=A_OUT3。编码器接口U24。端子编号以图纸为准，不能凭插头观察方向认定左右。实物PCB版本、端子压接及走线仍需连通性核实；没有PCB布局资料，不能仅由“最靠近电池”推断电气接错或更大电流。
- 固件对应Motor3为TIM1_CH3/PE13 PWM3、PE6 BRK3、PE2 DIR3、逻辑wheels[2]；四路使用同一PWM1/低有效极性/CCR=0启动配置。此前以右前轮推测Motor4的判断不应继续套用于U14；物理轮位仍按实际线束核对。
- 同电机换接口的对照强烈提高Motor3支路硬件、引脚映射、反馈与控制通道差异的优先级，但不是单凭现象确证某颗MOS损坏。若不同相分别存在上侧与下侧异常导通，接上电机后绕组可能才完成VCC至GND的过流回路；空接口不熔断不能排除桥异常。同一半桥上下管直通则通常无需电机也能构成短路，应区分这两类机制。其它待查包括U13/栅极电阻网络、连接器相线与霍尔/地脚误接，以及控制或反馈异常引发堵转。
- Q1重复损坏与支路熔断可能由同一次过流或瞬态引起，尚无波形不能确定先后顺序。总线R7图纸为0Ω，LTC4231 IN/SENSE被等效相连；官方手册明确此接法禁用断路比较器与模拟限流。故不能期待总开关必然先限流保护；Q1在上电爬升期间的线性SOA、U3及栅极网络也要核查，而非只替换Q1。保险丝熔断不保证半导体已获保护。
- 建议先完全断电、放电、拔下电机，对比故障/正常支路：三相输出分别到母线与GND的双向二极管读数、Q10–Q12各管D-S/G-S是否异常及CN4连通性；注意体二极管、并联电路和电容充电的影响，不用蜂鸣一次判短路。对异常器件需隔离确认。
- 后续带电定位应使用限流电源，先隔离MCU对PWM的驱动并在U13侧确认ENABLE(27)/BRAKEZ(28)高；避免将外加高电平与MCU推挽输出硬顶。即使硬件禁能仍过流，优先查桥/接线/驱动损伤；只在接回控制后发生，再查PE13到U13的实际波形、霍尔/编码器及旧固件自检。不要把直接接6S反复更换Q1或保险丝当诊断方法。TVS全拆后瞬态钳位缺失也需在最终硬件修复时重新评估。
- 官方依据：A4931第9页ENABLE低驱动，高超过约3ms关闭栅极，BRAKEZ高允许待机；https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf 。LTC4231第7页IN/SENSE接法、第11/16页浪涌与SOA要求：https://www.analog.com/media/en/technical-documentation/data-sheets/ltc4231.pdf 。AON6884官方为40V双N MOS：https://www.aosmd.com/sites/default/files/res/datasheets/AON6884.pdf 。本轮未取得实物阻值、栅极波形或电流记录，以上为优先排查路径，不是已确认根因。

## 26. 2026-10-18 用户确认故障电机接口功率MOSFET击穿

- 基线仍为d76a6f1039b75f0ee411c926c02863b951d67668工作树。本轮用户明确找到故障：U14对应电机接口有一颗功率MOSFET击穿。将第25节的功率桥损伤假设更新为用户实测确认；具体Q位号、上/下侧及检测数值未提供，不擅自指定Q10/Q11/Q12中的某一管。
- 该事实支持故障跟随Motor3接口而非跟随同一电机/线束的观察。接电机后绕组可能参与过流回路，但未取得具体桥臂失效组合，不能认定已证明某条精确短路路径。
- Q1 D-S短路及U14熔断仍为用户确认事实；支路MOS损坏可能导致过流并使Q1连带损坏，但损坏顺序和最初诱因尚未确定。不能断言所有BAT/I2C异常或MOS初始击穿均由同一原因引起。
- 修复建议为检查并更换已损MOS，同时检查该路其余桥管、U13栅极驱动与栅极电阻网络，及总开关U3/Q1网络；随后先在限流条件、输出关闭状态下验证，再恢复电机运行。第23/25节记录的18V TVS与6S电压不匹配、总线R7=0Ω的保护边界仍需处理，不因找到一个损坏器件便视为这些问题已解决。
- 本轮只更新故障记录，未改固件、构建或烧录。尚无更换该MOS后恢复正常的实机反馈。

## 27. 2026-10-18 用户反馈修改后自检不吸球，PWM5测不到波形（定位中）

- 基线仍为d76a6f1039b75f0ee411c926c02863b951d67668工作树。用户用示波器报告PWM5没有PWM、自检不吸球；尚未取得持续高/低电平、BAT读数、其它四轮自检状态及运行时寄存器。不能把无波形直接等同于TIM11未启动或确认板级故障。
- 逐项核对当前差异：第24节修改TIM1、PE9/11/13/14及四轮BRK，未修改TIM11、PB9或吸球代码；action.c/misc.c/dribbler_control.c相对当前HEAD无工作区差异。MX_TIM11_Init仍完成Base/PWM初始化、CH1 PWM1低有效、ARR6719/CCR0及HAL_TIM_PWM_Start，随后配置PB9 AF3。TIM11时钟由HAL_TIM_Base_MspInit开启，init_dribbler释放BRK5并启计数器。HAL_PWM_Start返回值当前未检查；尚无其实际失败证据，不添加无依据的重复启动补丁。
- 当前HAL的Base_Init/PWM_Init均初始化ChannelState为READY，故不能以“先Base_Init再PWM_Init”直接认定通道状态异常。项目内未发现其它PB9重新配置或TIM11 Stop调用。
- MODE3的do_robot_run仍在LED延时2秒后do_dribbler(1)，2秒后do_dribbler(0)。命令本身只设目标并先输出0，实际非零CCR由独立BoardUpdate任务每100ms的dribbler_update产生；该任务在启动期也运行。无线超时停吸球只在NORMAL/CRAY执行，不应覆盖SELFTEST_MODE。
- 两类明确的软件归零路径：battery无效/低于6S阈值19.8V持续约5秒后power_quiet锁存，board_actuators_disabled使自检返回并清负载；另一类是非零吸球命令下连续约1秒没有FG边沿，stalled锁存、吸球CCR清零，必须显式停命令才解除。后者可能让示波器只在自检最初不到1秒看到PWM，不能由测量较晚的静态高电平排除此前有输出。
- 区分办法：在吸球自检窗口观察TIM11->CCR1、CR1.CEN、CCER.CC1E、PB9的MODER/AFRH，并结合g_battery_diag.quiet、BAT有效性。计数与通道均启用且CCR0，优先查命令/保护；CCR在0与ARR之间但MCU脚无波形，再查复用、时钟/计数及引脚电气状态。低有效零占空比正常为持续高，不应为证明有输出强制给非零PWM或取消保护。
- 已询问用户当前BAT电压、自检四轮是否仍转、PWM5持续高还是低。等待实测以确定修复点，本轮未修改固件行为。
- 复跑tests/test_board_migration.py，4S/6S全部通过，包含吸球PI产生非零CCR、FG缺失停机及全局禁用；tests/test_motor_startup.py通过。宿主模拟不覆盖实机PB9电压、TIM11寄存器及任务执行现场，不能宣称已修复。

## 28. 2026-10-18 BAT正常、四轮会转、PWM5持续高：增加吸球历史诊断

- 用户补充实测：BAT=22.8V，自检四个行走轮仍会转，PWM5一直高电平。这降低当次全局quiet/关机锁存的可能性；持续高符合当前低有效PWM的零占空比，但不单独证明TIM11计数/通道/复用正常，也不证明从未短暂输出。
- 本轮复核实际HEAD已为afe2c7720c11c6d2a9a62aed45d02643eef60c53（fix the bug of suddent move of motor），已包含前述启动修改；第27节沿用d76a6f1描述不再作为当前提交基线。保留用户已有AGENT.md工作区记录及全部提交内容。
- Core/Inc/dribbler_control.h新增公开诊断类型dribbler_diag_t及volatile全局g_dribbler_diag，实现在dribbler_control.c。只记录现有控制路径，不新增PWM强制输出、重复初始化、自动重启、保护豁免或参数修改。
- requests/run_requests/rejected_requests记录合法命令调用及因执行器禁用被拒绝的非零请求；updates记录实际100ms速度环更新次数。command/target_rpm/rpm/duty/stalled/disabled发布当前控制状态，fg_edges为累计FG边沿。
- nonzero_outputs表示速度环写入非零比较值的累计次数，不代表已经用示波器测到PWM。last_nonzero_ccr/cr1/ccer及last_nonzero_ms保存最近一次非零比较值对应的寄存器快照；普通停止命令不会擦除历史。pwm_ccr/arr/cr1/ccer为最近一次命令/速度环执行时的当前快照，不是连续硬件采样。
- no_fg_stops在FG缺失首次使stalled置位时递增，该命令持续停机期间不反复计数；正常do_dribbler(0)仍按原实现清stalled，但历史计数保留，便于在2秒自检结束后判断是否已触发过反馈保护。诊断随MCU复位清零，不支持写诊断变量改变控制。
- Watch判读：run_requests=0先查MODE3命令路径；run_requests>0而updates不增长查BoardUpdate调度；rejected_requests>0结合disabled/g_battery_diag查全局禁用；nonzero_outputs>0表明软件曾写非零比较值，结合last_nonzero_cr1 bit0及last_nonzero_ccer bit0判断当时计数和通道是否启用；no_fg_stops>0表明至少一次缺反馈停机。若这些正常却PB9仍持续高，再查PB9 AF3/MODER、计数器实际变化、探测点及电气状态。
- tests/test_board_migration.py新增drib_diag用例，覆盖命令与更新、非零比较值/使能快照、无FG停机、停止后历史保留及全局禁用拒绝请求。4S/6S全部通过；test_motor_startup.py、test_startup_music.py通过。尚无本轮诊断固件实测，不宣称根因已确认或吸球已修复。
- 第28节构建验证：Keil编译/链接及HEX生成成功，0 Error(s)、0 Warning(s)，日志MDK-ARM/tmp/dribbler_diag_build.log；git diff --check通过。尚未烧录。

## 29. 2026-10-18 PWM5诊断截图：已写非零CCR，无FG后停机

- 当前源码基线afe2c7720c11c6d2a9a62aed45d02643eef60c53及第28节诊断工作树，保留现有改动。本轮用户提供Keil Watch的g_dribbler_diag截图，以下为实际读数：requests=2，run_requests=1，rejected_requests=0，updates=0x78(120)，no_fg_stops=1，nonzero_outputs=0xA(10)，fg_edges=0；command/target_rpm/rpm/duty均0，stalled=0，disabled=0。
- 当前pwm_ccr=0，pwm_arr=0x1A3F(6719)，pwm_cr1=1，pwm_ccer=3。历史last_nonzero_ccr=0xBE2(3042)，last_nonzero_cr1=1，last_nonzero_ccer=3；last_nonzero_ms=0x184B(6219ms)，last_request_ms=0x1C8A(7306ms)。3042/(6719+1)约45.27%逻辑驱动占空比，低有效意味着对应低电平时间比例，并非45%高电平。
- 这些记录支持：启动请求到达，速度环执行并至少10次写入非零CCR；最近一次非零时CR1.CEN=1、CCER.CC1E=1且CC1P=1。没有FG边沿并发生一次无反馈停机；当前stalled=0不能否定历史故障，因为随后正常停止命令会清stalled但不清no_fg_stops。当前CCR0与自检结束的停止状态一致，不能据此断言从未下发PWM。
- 尚不能由寄存器快照证明PB9实际曾输出波形，也不能把无FG反馈本身视为最初原因；电机未被驱动或未转同样会无FG。候选范围移至PB9实际复用、TIM11输出模式/时钟计数、测量窗口/测点、连接与引脚电气状态，暂不取消无反馈保护或强行延长输出。
- 核对当前源码，PB9仍配置为GPIO_MODE_AF_PP、GPIO_AF3_TIM11。后续I2C2使用PB10/PB11、SPI2使用PB13/14/15；软件I2C宏也只操作PB10/PB11，未发现该路径覆盖PB9。源码配置不替代实机MODER/AFR读数。
- 已请求读取(GPIOB->MODER >> 18) & 3（期望2）、(GPIOB->AFR[1] >> 4) & 15（期望3）、TIM11->CCMR1（CH1应CC1S=0、OC1M=6），并说明测点为MCU PB9还是功率板PWM5。这些复用配置自检结束后仍可读取，暂不要求再次烧录。示波器需覆盖启动后约1秒的实际驱动窗口，静态停止后的高电平不能否定该窗口内波形。
- 本轮只更新分析记录，未修改控制代码、构建或烧录；PWM5引脚无波形的最终原因仍待上述实机证据。

## 30. 2026-10-18 PB9模式/复用与TIM11 PWM模式实测正常，测点在功率板

- 用户实测：(GPIOB->MODER >> 18) & 3 = 2；(GPIOB->AFR[1] >> 4) & 15 = 3；TIM11->CCMR1 = 0x60。符合PB9复用模式、AF3/TIM11及CH1输出PWM1（CC1S=0、OC1M=6）。测量位置明确为功率板标注PWM5处，尚未直接测MCU PB9。
- 结合第29节last_nonzero_ccr=3042、ARR6719、CR1=1、CCER=3，已取得命令、非零比较值、输出使能及引脚复用的支持证据。尚缺实际计数时钟/变化及PB9波形证据，因此不宣称MCU引脚已输出，更不能由功率板测点高电平认定软件没有写PWM。
- 重新视觉核对powerboard_schematic.pdf第2页：Control-Board Connector CN1第28脚为PWM5，直接连接Dribbler Driver U9的ENABLE第27脚，RN4提供5.1kΩ至3V3上拉；CN1第24脚为BRK5，26脚为DIR5。没有在图中看到PWM5路径上的逻辑反相或隔离器。主控板的PB9至板间连接器实际走线图尚未找到，不凭同名网络替代端到端连通性核实。
- 推断：若主控到功率板该路开路/接触不良，功率板侧仍可被RN4拉高，与用户持续高的现象相符；这是待测假设，不是已确认断线。过去功率器件故障可能涉及信号端，但尚无PB9或U9损伤证据。
- 后续对照：用示波器同一启动周期同时测MCU PB9与功率板CN1-28/U9-27，公共地，覆盖音乐后实际吸球启动约1秒窗口；可用下降沿单次触发避免仅看停止后高电平。PB9有而功率板无，查板间连接/焊点/端子/实际映射；两端都有则继续查BRK5、驱动供电、桥管及FG回路；PB9也无则查TIM11->CNT在运行中是否变化、APB2时钟和调试冻结及引脚负载/损伤。断电放电后检查PB9到CN1-28/U9-27连通性，避免带电用电阻档。
- 本轮只补充定位记录，未改固件、未重新构建/烧录；此前测试结果不作为端到端线路或引脚完好证明。

## 31. 2026-10-18 用户明确：四轮防突动修改前、已有FG闭环时吸球正常

- 用户补充：修改代码前吸球能够正常转动；已更换吸球电机驱动芯片但仍不转。进一步明确正常阶段是“本次四轮开机防突动修改之前（已有FG速度闭环）”。此证据要求优先核对软件回归，不能把问题归为新增FG闭环，亦不能由驱动芯片替换推定所有硬件必然完好。
- 当前基线afe2c7720c11c6d2a9a62aed45d02643eef60c53及吸球诊断工作树。以相邻提交d76a6f1与afe2c77做实际源码比较：Core/Src/action.c、misc.c、dribbler_control.c、Core/Inc/cfg.h完全一致；TIM11初始化及PB9复用配置也一致。d76a6f1是选用的相邻源码比较点，不等于已取得用户最后正常烧录的AXF/HEX或确认其精确提交号。
- 此间真正启动变化为TIM1的GPIO高暂存/延后AF、零CCR/UPDATE及四通道启动顺序、PRIMASK保持、GPIOE四轮BRK默认高，以及robot初始化调用名替换；freertos的差异为音乐末音间隔处理，BoardUpdate/吸球调度未变。motor_hold_startup当前对四轮的控制操作与原init_motor相同，均为BRAKEZ高、DIR0、CCR0、is_motor_run0。
- 回溯更早84c6b4f迁移前确有固定约50/55%占空比改为PI爬升的历史差异；但用户确认本次正常版本已含FG闭环，故不把这项更早历史当作本轮根因，不据此调高占空比、取消无反馈停机或恢复开环。
- 搜查应用时钟/计数操作：未发现新增关闭/复位TIM11、修改PB9或写整个APB2寄存器的调用。HAL时基使用TIM14，并非TIM11。未找到四轮修改直接破坏TIM11的源码路径；这只是源码审查结果，不能据此排除实机时序、电源或其它间接软件影响。
- 现有快照只能证明非零CCR与CEN/CC1E配置，不能证明TIM11实际计数或PB9波形。优先补足运行状态下TIM11->CNT是否变化的证据；已请求用户保持CPU运行连续刷新CNT，暂停下的固定值不作判断。此前功率板测点与MCU PB9直接波形对照仍未取得。
- 本轮保持四轮防突动修复及现有吸球保护，不做无依据的重复初始化、调参数或恢复可能产生启动轮动的旧启动路径；仅更新调查记录，未修改固件行为、构建或烧录。尚未确认吸球回归根因。
- 第31节后续实测：用户确认CPU运行时TIM11->CNT会变化，补足定时器实际在计数的证据。结合已有CCR/CCER/CCMR1/MODER/AFR，当前内部输出配置与计数均符合预期；仍不能用这些值替代PB9物理波形。下一项直接测MCU PB9，在通讯灯由亮转灭、do_dribbler(1)开始时捕获；若PB9有PWM而板侧没有则定位连接段，若PB9无则继续查MCU输出/负载。此更新不宣称软件回归已排除或硬件根因已确认。

## 32. 2026-10-18 用户直接测PB9：一次拉低后回高，未见连续PWM

- 用户新增物理观测：MCU PB9只有一个周期的拉低，随后拉高，没有观察到持续PWM。此前仅功率板测点的证据边界据此更新；MCU输出端也需定位。但PB9仍连接外部负载，尚不能单独由此认定MCU损坏或完全排除外部钳位。
- 当前源码基线仍afe2c7720c11c6d2a9a62aed45d02643eef60c53及诊断工作树；复查TIM11初始化/使用和所有OPM调用，未找到给TIM11设置单脉冲或周期重启的源码。TIM6/7/9的OnePulse调用不是TIM11。原截图非零输出时CR1=1；本地stm32f407xx.h定义OPM=0x8，因此该快照单脉冲位为0。不能仅凭一次低电平就宣称TIM11进入单脉冲模式，也不能用重启定时器作为根因修复。
- 配置为168MHz定时器时钟、PSC0、ARR6719时，期望周期40μs。此前最后一次非零CCR3042对应低电平约18.1μs；该计算来自配置，不是实测频率。当前证据未包含PSC/APB2运行值与实测脉宽。
- 已请求用户提供那次低电平的μs/ms持续时间，以及示波器捕获时MCU是否始终运行、没有断点/暂停。还需带时基/电压刻度的波形区分真正孤立的微秒脉冲，与毫秒/秒级驱动窗口在长时基/采样不足下显示的包络。后者为待验证测量假设，不据此否定用户观测。
- 尚无新的控制行为改动或构建/烧录。保留无FG保护，不将无反馈停机当作PB9无连续波形的已确认最初原因。

## 33. 2026-10-18 PB9一次下陷约22ms、约2V，CPU始终运行

- 用户补充：此前所称PB9的单次拉低持续22ms，电平约2V而非0V，测量时MCU始终运行，无断点/暂停。22ms不是预期25kHz/40μs的单个周期；调试暂停假设缺少本次支持。尚未取得示波器时基、采样率、耦合/平均设置和带刻度截图。
- 2V可能是实际异常电平，也可能是未解析的PWM平均/显示包络，不能仅凭该描述认定MCU GPIO已损坏。此前历史CCR3042/6720约45.27%低电平驱动，若高3.3V/低0V，则周期平均约3.3*(1-3042/6720)=1.806V；接近2V是值得排除的测量假设，不证明该22ms现象就是平均值或完整驱动窗口。
- 当前PB9源码仍AF_PP推挽、NOPULL，尚未取得运行时OTYPER位。已请求读(GPIOB->OTYPER >> 9) & 1，期望0；与此前MODER=2、AF=3组合判断实际输出类型。未通过提高GPIO速度、强制占空比或重启定时器试错。
- 再核对官方A4931 Rev9第5页：控制输入VIN(1)最低2V，VIN(0)最高0.8V，输入电流在所列条件下约±1μA。因此若微秒尺度实际低电平也停在2V，它不是保证有效的低电平，能导致无法正常使能；应优先查推挽输出能力、负载/短接/地参考，而不是仅换A4931或改PID。资料：https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf 。
- 已请求将示波器改为DC耦合、20μs/div、至少10MSa/s、关闭平均/高分辨率，在吸球启动窗口看PB9是否有0～3.3V、约40μs周期的波形；探头倍率与通道倍率需一致、地接MCU GND。如果此设置仍确认低仅2V，再断电隔离PWM5信号线并测MCU侧，保留其它供电/电池监测连接与公共地，避免拔整块功率板使BAT无效从而触发quiet、污染比较条件。
- 本轮只更新诊断记录，未改变固件行为或重新构建；PB9异常的最终原因仍待有效波形与输出类型实测。

## 34. 2026-10-18 确认PB9有40μs周期方波，但幅度仅2～3.3V

- 用户按DC耦合、20μs/div复测，明确看到周期40μs、低约2V/高约3.3V的连续方波。更新第32/33节的“未见连续PWM”观察：已取得MCU PB9存在25kHz PWM的直接证据，频率与当前配置吻合；剩余关键异常是电平幅度。不能继续将现象称为仅一个PWM周期，也不能将2V直接作为已证实的正常PWM平均值。
- 结合官方A4931输入低电平上限0.8V（第33节来源），若以正确公共地参考测得的实际低电平为2V，则不能保证ENABLE低有效，足以解释电机未使能及FG=0后软件超时停机；原始电平异常的原因仍未确证。
- 本轮不改PID、PWM频率、FG保护或增加外部强下拉。需要确认运行时(GPIOB->OTYPER >> 9) & 1为0（该值此前尚未回复），及探头地与MCU GND一致。源码AF_PP与实际输出类型不可混为同一实测证据。
- 优先隔离试验：断电后只断开PB9至功率板的PWM5信号，保留公共地和其它供电/电池监测连接，再运行相同自检并测MCU侧。功率板PWM5应由原RN4上拉保持禁用，不能把它短接到地。MCU侧恢复0～3.3V，则定位外部PWM5线路/负载/错接或反向驱动；空载仍2～3.3V且推挽及测量确认正常，再查MCU PB9输出级、主控板本地负载/焊接问题。空载异常本身也不能未经隔离检查便断言整颗MCU损坏。
- 图纸RN4为5.1kΩ上拉至3.3V，其静态拉低负载约0.65mA；正常上拉不是把健康推挽低电平钳在2V的充分解释。需要检查额外负载或驱动冲突，而非仅更换已经换过的A4931。
- 当前源码基线afe2c7720c11c6d2a9a62aed45d02643eef60c53及既有诊断工作树。本轮只记录实测与定位步骤，未修改固件、构建或烧录，未声称根因已最终确认。

## 35. 2026-10-18 RN4上拉疑问与整束排线的USB隔离测试

- 用户确认运行时(GPIOB->OTYPER >> 9) & 1=0，PB9为推挽；说明PWM5与其它信号为整束排线，单独断开一根不便操作。此前要求保留其余线路并单断PWM5的建议未考虑实际连接便利性，本节改为整束拔下、主控独立测试，不要求剪线或挑针。
- 用户确认主控可USB独立供电。正常固件在功率板断开后因BAT无效会quiet，故不要求用正常自检固件做整束断开的比较，避免测不到输出却误判PB9。
- 用户询问RN4是否把低电平抬高。正常5.1kΩ至3.3V的拉低电流约0.65mA，不足以解释健康推挽输出低电平2V；若RN4装错为小阻值、焊桥或连线错误，则仍可能是异常负载源，不能直接排除RN4。断电在板比较PWM5到3.3V电阻与其它正常通道，需考虑并联器件影响。
- ST官方STM32F405xx/407xx数据手册I/O输出特性：2.7V<VDD<3.6V、8mA吸收电流条件下VOL最高0.4V，支持上述正常轻载不应有2V低电平的判断；不是对当前受损引脚的实测保证。来源：https://www.st.com/resource/en/datasheet/stm32f407ve.pdf 。RN4阻值源于用户原理图第2页，实装阻值尚未实测。
- 新增tests/build_pb9_usb_test.py：读取当前main.c，只在tmp/pb9_usb_test生成独立主入口，调用HAL_Init/SystemClock_Config/MX_TIM11_Init，设CCR1=(ARR+1)/2后WFI等待；不进入机器人初始化、RTOS、四轮、踢球或FG控制。保留原PB9 AF3/PWM1/低极性配置，预期持续25kHz/50%。正常源码、正常.uvprojx与正常HEX不修改。
- 独立Keil工程转换源码/包含路径为绝对路径，输出单独位于MDK-ARM/output/pb9_usb_test/PB9_USB_Test.hex。操作说明为MDK-ARM/PB9_USB_TEST.md。该镜像仅用于整束功率板线已断开、USB供电的主控；接回功率板前恢复正常固件并断电接线。
- 空载恢复0～3.3V支持排线/功率板负载问题，仍2～3.3V则继续查主控本地负载、PB9输出级及测量，不直接认定整颗MCU损坏。测试未实机执行，不声称RN4或MCU已被最终定责。
- 第35节构建结果：独立PB9_USB_Test工程Keil构建成功，0 Error(s)、0 Warning(s)，HEX已生成；日志MDK-ARM/tmp/pb9_usb_test/pb9_usb_test_build.log。正常main.c与正常SRC_Firmware.uvprojx无差异，正常输出目录未被测试工程使用；git diff --check通过。未烧录。

## 36. 2026-10-18 R39与吸球PWM5关系核对

- 用户询问R39是否导致当前吸球PWM5异常。重新放大核对powerboard_schematic.pdf第3页：R39=0Ω，属于Motor2/U16，连接U16第25脚CLD至GND；不是PWM2输入上的电阻，更不是PWM5通路器件。第2页吸球U9的同功能电阻为R25=0Ω，同样将第25脚CLD接地；PWM5连接第27脚ENABLE、RN4上拉。
- A4931官方Rev9第10页Lock Detect Function明确：CLD接GND可禁用硬件堵转检测；R39/R25的0Ω接地是该手册支持的禁用接法，不应误判为把PWM输入短路。来源：https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf 。这也意味着该硬件堵转保护不能被假定有效，原软件FG超时保护是另一条独立路径。
- 在实物与该图一致且无焊桥/错接的前提下，R39正常连接不能解释吸球PB9的2～3.3V电平；即使讨论吸球R25，其CLD功能也不是PWM输入电平整形。异常焊接或不同版本实物需要单独核对，不据原理图宣称器件实物必然完好。
- 独立PB9 USB测试仍为当前区分主控输出与功率板负载的有效下一步。本轮只更新记录，未改固件、硬件或重建；尚未取得用户执行独立测试后的结果。

## 37. 2026-10-18 独立PB9测试工程打开时请求缺失CMSIS 4.5.0

- 用户截图确认Pack Installer报错：Install By Pack ID，ARM:CMSIS:4.5.0，Pack not found。前次仅CLI编译成功未覆盖GUI打开时的RTE包选择检查，不能以构建通过声称工程打开无问题。
- 检查独立与正常.uvprojx，正常工程原有RTE组件CMSIS/CORE/Cversion4.3.0，package指定ARM.CMSIS4.5.0，targetInfo仍为SRC_Firmware；生成器照搬此条导致测试工程打开后请求旧包。本机E:/Keil_v5/ARM/PACK/ARM/CMSIS仅有4.2.0，设备包Keil.STM32F4xx_DFP1.0.8已安装。
- 测试实际源码与IncludePath使用仓库Drivers/CMSIS的核心/设备头文件，不依赖该RTE自动加入的文件；RTE/files为空。已修改tests/build_pb9_usb_test.py，仅从测试工程删除多余CMSIS/CORE的RTE选择及空::CMSIS组，处理剩余targetInfo名，保留已安装的设备包及全部源文件。正常main.c、正常.uvprojx及固件行为不修改，也不修改本机全局包安装。
- 重新生成并静态验证：测试XML中无4.5.0选择，无CMSIS RTE组件，设备包仍为Keil.STM32F4xx_DFP.1.0.8，全部源文件路径存在。随后执行独立工程全量重建；GUI打开尚未实测，建议关闭旧测试工程时不保存旧配置，再重开生成的同一路径工程，避免内存中的旧工程覆盖修复。
- PB9_USB_TEST.md同步增加此错误处理说明。台架测试条件仍为整束功率板排线拔下、USB独立供电、恢复正常固件后才能断电接回功率板。
- 第37节全量重建完成：PB9_USB_Test为0 Error(s)、0 Warning(s)，AXF/HEX已生成，代码6974字节；日志MDK-ARM/tmp/pb9_usb_test/pb9_usb_test_packfix_build.log。git diff --check通过，正常main.c/.uvprojx仍无改动。尚未验证用户重新打开GUI或烧录实机。

## 38. 2026-10-18 用户确认USB独立测试控制板正常

- 用户反馈“独立测试，控制板没问题”，表明用户已执行第35/37节测试并认为主控空载输出正常。用户未提供本轮电平数值/截图；据此支持PB9具有正常输出能力，将排查优先级转向排线、功率板输入及接入负载，不扩展为整块主控所有功能或正常固件全部行为已验证。
- 与接入功率板时40μs、2～3.3V方波对照，异常负载/输出冲突是当前重点。正常RN4 5.1kΩ上拉不是足以抬高健康推挽低电平的理由，但RN4装错、焊桥或错误阵列连接仍待核实。
- 优先断电、拔排线检查U9输入邻脚：ENABLE27是否与DIR26或BRAKEZ28近零阻短接/焊桥；后两路由主控输出高时，若与PWM5短接可能形成两个推挽输出对抗，导致低电平抬高。该机制与现象相符但未经实物确认，不能宣布已找到焊桥。用户已换U9并不能排除更换后的焊接或邻脚连通异常。
- 随后核对RN4实装阻值/方向及PWM5到3.3V阻值，与正常PWM通道比较，并考虑在板并联路径。检查CN1第28脚至U9第27脚及板间排线映射，确认没有与电源、DIR5、BRK5等错接，也核查公共地连通。
- 接回功率板前先恢复正常SRC_Firmware固件，再断电接线；独立测试镜像持续50% PWM且不运行FG/电池保护，不能用于整车运行。保持现有正常固件保护，不新增开环或强输出补丁。
- 基线仍afe2c7720c11c6d2a9a62aed45d02643eef60c53及诊断工作树。本轮只记录实测和下一步板级检查，未改代码、构建或烧录；具体功率板/线束故障点仍待用户测量。

## 39. 2026-10-18 用户确认R14/R17由470Ω换成0Ω

- 用户提出是否因3V3/5V模块470Ω改0Ω导致PWM5异常；询问R14或R17后回答“都换了”，记录为两颗均已更换。尚未取得5V_BAT与功率板3V3实测值、纹波或更换前后对照，不推定其电源轨正常，也不直接认定此改动就是根因。
- 重新放大powerboard_schematic.pdf第1页：R14在母线VCC至U6 LMR23630ADDA的VIN（第7脚）/EN（第5脚）输入支路，后有C16/C17；R17在5V_BAT至U7 LT3080EST输入第3脚支路，后有C23/C25。两者串在主输入供电路径，不是VCC内部偏置输出上的电阻，也不是反馈设定电阻。
- 5V设定由R15=88.7kΩ/R16=22.1kΩ反馈分压决定；3V3设定由LT3080 SET脚R21=330kΩ及其10μA基准决定。TI LMR23630数据手册§9.2.2.2、ADI LT3080 Output Voltage节支持此语义：https://www.ti.com/lit/ds/symlink/lmr23630.pdf ；https://www.analog.com/media/en/technical-documentation/data-sheets/lt3080.pdf 。仅把串联输入电阻改0Ω不等于短接稳压芯片输入输出，也不直接改变输出电压设定。
- 图纸470Ω标注本身需要核对：普通470Ω在10mA时压降4.7V，在5V到3.3V LDO输入路径甚至忽略LDO压差也仅有约3.6mA压差预算，与标注800mA供电能力不符。不能直接建议换回普通470Ω；可能存在图纸单位/器件性质差异（例如磁珠高频阻抗标注），但尚无BOM/实物证据，不将此可能当成设计事实。
- 0Ω会改变原有输入阻抗和滤波/瞬态条件，焊接也可能引入桥接；如果因此出现电源异常，可能间接影响负载/PWM，但尚不能由该更换动作单独解释健康推挽PB9接入后低电平2V。正常5.1kΩ上拉即使电源略升也不能自动解释此幅度，应结合实际负载、错接和共地检查。
- 下一项先测功率板5V_BAT与3V3相对本板GND，确认接入主控后仍稳定，并核对两板GND连通/压差；若3V3意外接近5V，应停止连接主控，查LT3080引脚与输入输出桥接/器件损伤。用户尚未回复电压数值，本轮只更新记录，不修改固件、硬件或重新构建。

## 40. 2026-10-18 用户确认电源/焊接/器件检查正常，补查PB9带载能力

- 用户反馈：功率板5V_BAT/3V3测量均正常；仔细检查无连锡；MOSFET/A4931检查均无损坏。记录为用户检查结果，未提供具体电压、纹波、芯片检测方法/阻值，不直接扩展为所有外部网络、电源瞬态或器件功能均已排除。
- 此反馈降低明显电源电压异常、已知功率器件击穿和可见焊桥的优先级；R14/R17改0Ω本身仍无直接导致PWM低电平2V的证据，不建议无依据恢复普通470Ω或改控制参数。
- 已进一步询问独立测试PB9的实际低/高电平，以及断电拔整束排线后的PWM5至3V3/GND/DIR5/BRK5阻值，与正常通道比较。当前尚未收到读数。
- 修正此前结论边界：第38节“空载正常支持主控输出能力”不能当作PB9具有正常带载能力的确证。损伤后弱下拉也可能空载正常、接正常5.1k上拉后低电平升高。因此不能只因空载测试正常就把MCU输出级排除。此前用户也仅说控制板没问题，未给空载电平数值。
- 提出优先受控负载试验：保留USB独立测试固件、整束功率板排线拔下，在PB9与主控自身3.3V间接一颗已知4.7kΩ～5.1kΩ电阻，模拟正常RN4上拉（约0.65～0.70mA拉低负载），测PB9相对主控GND低电平。不要直接短接至3.3V，也不接5V。接线时断电。
- 加上述已知轻载后仍接近0V，才进一步支持PB9在该条件下带载正常，后续重点回到功率板/排线额外负载或错接；若因此升到约2V，说明异常可在主控侧复现，重点查PB9弱驱动/本地连接及测量，不立即认定具体损坏器件。低电平预期参考ST数据手册I/O特性，既有来源：https://www.st.com/resource/en/datasheet/stm32f407ve.pdf 。
- 该试验不需改变固件行为或再次构建。尚未实际实施，不宣称PB9损坏已确认；正常固件与无FG保护保持现状。源码基线仍afe2c7720c11c6d2a9a62aed45d02643eef60c53及诊断工作树。本轮仅更新记录。

## 41. 2026-10-18 按用户要求只对比PWM配置，提供CubeMX修改指引

- 用户表示外接电阻/阻值实验不便，要求先比较PB9通道与其它电机PWM配置，若不同提供CubeMX改法，由用户自己修改，明确禁止直接修改配置。本轮只读Core/Src/tim.c、SRC_Firmware.ioc、HAL实现及官方手册；只更新本记录，不改.ioc、tim.c、GPIO或其它固件配置，不构建/烧录。
- 当前基线afe2c7720c11c6d2a9a62aed45d02643eef60c53及既有诊断工作树。四轮为TIM1_CH1–4/PE9/11/13/14，吸球为TIM11_CH1/PB9。两者APB2定时器时钟168MHz、PSC0、向上计数、ARR6719、DIV1，预期25kHz；PWM1、低有效、初始Pulse0、Fast Disable一致。
- 运行时GPIO两者均AF_PP推挽、NOPULL、GPIO_SPEED_FREQ_LOW。TIM1为AF1，PB9/TIM11为AF3是外设映射要求，不能为了相同而将PB9改成AF1。
- 差异1：TIM1 AutoReloadPreload ENABLE；TIM11 DISABLE；.ioc的TIM1.AutoReloadPreload/TIM11.AutoReloadPreload一致证实。
- 差异2：HAL_TIM_PWM_ConfigChannel默认置对应OCxPE，TIM1未清除，故四路CCR预装载启用；TIM11初始化后显式__HAL_TIM_DISABLE_OCxPRELOAD(&htim11,TIM_CHANNEL_1)，.ioc亦为TIM11.OC1Preload_PWM=DISABLE。用户读到CCMR1=0x60与PWM1/OC1PE0一致。
- CubeMX指引：Timers→TIM11→Parameter Settings→Counter Settings，将auto-reload preload设Enable；PWM Generation Channel1的Output compare preload/Preload设Enable。保持PSC0、CounterModeUp、ARR6719、ClockDivisionDIV1、PWMMode1、Pulse0、PolarityLow、FastDisable。GPIO Settings保持PB9 AF Push Pull、NoPull、Low。菜单英文大小写随CubeMX版本可能不同。
- 若用户选择统一配置，生成后应确认TIM11 AutoReloadPreload=ENABLE，且不再生成清CCR预装载的__HAL_TIM_DISABLE_OCxPRELOAD。开启后正常运行中CCMR1应约0x68，CR1至少ARPE位0x80置位（计数运行时通常0x81）；这是校验建议，不是在本轮代改寄存器。预装载使CCR/ARR更改在更新事件生效，主要影响更新同步，不能据此宣称已解释或修复2V低电平。官方RM0090：https://www.st.com/resource/zh/reference_manual/DM00031020.pdf 。
- 其它差异：TIM11初始化时启动PWM，TIM1在音乐/IMU后建立零输出并由GPIO高转AF；TIM1为高级定时器，存在互补/死区/主输出等额外配置，TIM11没有对应设置。保持这些硬件/启动差异，不把它们当成必须复制的TIM11参数。
- CubeMX重新生成风险：会在MX_TIM1_Init末尾恢复自动HAL_TIM_MspPostInit(&htim1)，这会覆盖第24节延迟AF接管策略。生成后烧录前需保留GPIO高暂存并确认AF仍只在motor_prepare_pwm零输出建立后接管；同时确认TIM11 USER CODE内原HAL_TIM_PWM_Start仍保留。用户本次只授权其自行改CubeMX，不在当前工作树自动处理该潜在生成结果。

## 42. 2026-10-18 区分推挽PWM上拉与开漏FG反馈

- 用户询问MCU可推挽输出时为何仍需“上拉+开漏”。复核当前TIM11 PB9仍为GPIO_MODE_AF_PP、NOPULL；用户此前运行时OTYPER9=0也证明该时刻推挽。因此不能把RN4存在等同于PWM5配置为开漏。
- PWM5方向为MCU→A4931 ENABLE输入；推挽负责正常运行时主动高/低驱动，功率板RN4上拉用于主控复位、未初始化或排线断开时给出默认高电平。结合A4931 ENABLE低驱动、高禁用/待机语义，此上拉有利于默认关闭，是本电路用途分析，不声称已取得设计者的原始意图说明。
- A4931 FG1/FG2方向为驱动芯片→MCU；官方Rev9第10页Hall State Reporting明确两脚为开漏输出，需通过上拉获得确定高电平。MCU通常配置为输入/EXTI，不是把MCU端也设为开漏PWM输出。资料：https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf 。
- 一般原理：推挽主动拉高/拉低；开漏只主动拉低，释放时由上拉拉高，适用于多器件共享低有效线和满足管脚耐压/供电条件的电平接口。开漏上升沿取决于上拉与线路电容，推挽通常更适合单向PWM。外部上拉与推挽可以并存，不要求为了使用上拉改开漏。参考ST AN4899 GPIO说明。
- 对当前故障：正常5.1k上拉只增加轻微拉低负载，不应使健康推挽低电平为2V；改开漏也不会自动修复弱下拉、输出冲突或异常负载。保持用户“配置由其在CubeMX修改”的要求，本轮只解释和记录，不修改配置或构建。

## 43. 2026-10-18 不应将整颗RN4当作单一PWM上拉直接拆除

- 用户询问是否可取下RN4。重新完整放大powerboard_schematic.pdf第2页RN4/R24/U9接线，确认RN4不是仅PWM/BRK上拉：它包含四个5.1kΩ独立电阻，图上左起依次为DIR5下拉、霍尔负端参考下拉、PWM5上拉、BRK5上拉。实际封装脚号/朝向必须核对实物，不能以图上左右位置当焊盘序号。
- 新确认的重要支路：U9 HA-/HB-/HC-（2/4/6脚）共接R24=10kΩ下端节点；R24另一端接U9 HBIAS，RN4第二支路将该参考节点以5.1kΩ接地。按HBIAS约7.5V，参考电压约7.5*5.1/(10+5.1)=2.53V。
- 若整颗RN4拆除且其它网络未补回，霍尔参考失去5.1kΩ下拉，可能经R24趋向HBIAS约7.5V，破坏霍尔比较条件；官方A4931 Hall输入共模范围0.2～3.5V、Hall脚绝对额定最高7V，不能将此状态当作正常可上电运行配置。实际节点取决于芯片输入/保护和状态，因此不声称一定恰好7.5V。资料：https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf 。
- 同时整颗拆除会丢失PWM5/BRK5默认高电平和DIR5默认低电平，不能用“MCU推挽”作为删除全部启动/参考偏置的理由。这是对前文只讨论PWM5上拉作用的补充，不改变此前单一正常5.1kΩ负载不足以解释2V低电平的判断。
- 如仅为诊断PWM5上拉，理论上只可隔离其对应单支路并保留另外三路；若整颗拆，则必须先补回其它所需电阻，尤其霍尔参考下拉，并确认启动默认电平。若怀疑排阻异常，可替换同规格、同脚位的4×5.1kΩ独立排阻进行验证，而不是空缺整颗后运行。未取得PCB封装/BOM，不提供未经核实的具体引脚操作。
- 本轮只记录分析，不修改固件配置、硬件或构建。当前仍无实际拆除/替换RN4后的测量，不能认定RN4已损坏。

## 44. 2026-10-18 用户CubeMX重新生成后检查与恢复生成回归

- 用户表示已在CubeMX修改并重新生成。当前HEAD仍afe2c7720c11c6d2a9a62aed45d02643eef60c53，保留既有诊断工作树及本次用户生成改动。按此前约定不代改用户的CubeMX外设参数。
- 实际确认：TIM11 ARR预装载已ENABLE，.ioc的OC1Preload_PWM已ENABLE，tim.c不再清OC1PE；原TIM11 USER CODE中的HAL_TIM_PWM_Start保留。PWM1、Low极性、PSC0、ARR6719、PB9 AF3推挽/NOPULL仍一致。PB9最大输出速度仍LOW，用户此次未设置High；不自动替其更改。
- 生成回归1：MX_TIM1_Init末尾恢复了HAL_TIM_MspPostInit(&htim1)，导致GPIO高暂存随后立刻被AF覆盖，破坏第24节已授权的延迟接管保护。先运行tests/test_motor_startup.py复现失败（断言该调用不在初始化中）；仅移除此多余调用，保留motor_prepare_pwm中零CCR/UPDATE/四通道启动完成后的AF接管。未改.ioc或用户TIM11参数。
- 恢复后启动测试、tests/test_board_migration.py（4S/6S全部，包括吸球诊断）、tests/test_startup_music.py均通过。宿主模拟不验证物理电平；当前没有预装载改动后吸球恢复/低电平恢复的实测。
- 生成回归2：Keil初次构建出现8个L6200E重复符号错误，原因是system_stm32f4xx.c被Application/User与Drivers/CMSIS同时列入。初步以集合比较文件列表未显示重复，但实际链接暴露重复；随后以规范化路径计数确认。移除新加的Drivers/CMSIS重复条目，保留原Application/User条目及其编译选项，不删除源文件。清理工程尾部多余空行使git diff --check通过。
- 用户生成的其它工程变化保留，包括Device显示由STM32F407VE改为STM32F407VETx及原编译器选择；源码文件集合没有丢失，去重后所有路径唯一。没有在本轮调整器件型号或全局包安装，正常工程原有CMSIS4.5.0 RTE选择仍保留；本轮没有验证GUI重新打开是否触发包提示。
- 修复后正常工程构建成功，Code54266，0 Error(s)、0 Warning(s)，AXF/HEX已生成，日志MDK-ARM/tmp/cubemx_regen_review_fixed_build.log。初次失败日志tmp/cubemx_regen_review_build.log保留作为回归证据。
- 使用tests/build_pb9_usb_test.py重新生成独立测试工程，同步用户当前TIM11参数，同时继续去除其旧CMSIS RTE冗余依赖并输出至独立目录。台架测试条件仍为整束功率板线拔下、USB独立供电；不得把测试镜像用于整车。正常/独立镜像本轮均未烧录。
- 第44节独立测试同步构建完成：PB9_USB_Test为0 Error(s)、0 Warning(s)，Code6962，AXF/HEX已生成；日志MDK-ARM/tmp/pb9_usb_test/pb9_usb_test_after_cubemx_build.log。该测试HEX已使用当前TIM11预装载配置，不再是修改前的旧镜像。

## 45. 2026-10-18 用户确认PWM5与BRK连锡，PB9输出问题已解决

- 用户明确表示无需继续排查，已发现PWM5与BRK连锡，并确认PB9输出问题已解决。将本问题状态更新为用户实测确认的硬件焊接故障；此前第40节所述目测无连锡是当时检查结果，现由该新证据修正。用户本句未注明BRK具体编号，不擅自补作已确认BRK5。
- 机理解释：PWM5与保持高电平的BRK输出短接时，PWM5的低电平阶段可能形成两个推挽输出对抗，使波形低电平被抬高；该机理与接板时25kHz/约2～3.3V、主控独立测试正常的观测一致。此前FG=0及软件无反馈停机是该情况下的后续现象，不应继续当作最初故障原因。
- 不将此次已确认根因归为R14/R17改0Ω、正常RN4上拉、TIM11预装载、GPIO速度或A4931损坏；这些此前均为待查项。用户未提供去锡后的具体电平/转速数值，不额外声称已完成全车或全部硬件保护验证，也不把本结论延伸为此前Q1/功率MOS/保险丝损坏的已证实唯一原因。
- 停止本问题的进一步诊断、参数试改和要求额外测试。保留用户本轮CubeMX设置，以及第44节已完成的四轮启动保护恢复和重复编译条目修复；用户未要求回退，故不自动恢复旧配置或删除测试工件。AGENT.md保留调查与已确认结论供后续参考。
- 本次只更新记录，未修改固件、重新构建或烧录。先前正常/独立测试镜像构建结果见第44节。

## 46. 2026-10-19 PWM连锡排除后仍不吸球：重新区分运行状态

- 用户确认PWM输出问题解决，但自检吸球环节仍不吸球；尚未明确完全不转、短暂抖动后停止，还是能转但方向/带球异常。第45节只结案PWM电平故障，不代表吸球整体功能恢复。
- 当前基线d17910f3fafc7b3340ac12fb01c70f20e1089832（fix the bug of PWM5），初始只有AGENT.md已有未提交修改。本轮只读核对robot.c、action.c、misc.c、dribbler_control.c、cfg.h及GPIO/TIM/EXTI路径。
- 当前MODE3命令仍为do_dribbler(1)、等待2000ms、do_dribbler(0)；init_dribbler将PE8/BRK5置高，运行方向输出为PE10/DIR5，PB9为TIM11_CH1低有效PWM，PD4/FG1上升沿回调计数。连续1000ms无FG会锁存停机，显式零命令又会清除stalled，因此自检后stalled=0不能排除本轮曾无FG停机，应看no_fg_stops历史计数。此前连锡期间的截图不能用作修复后当前状态。
- cfg.h仍存在历史记录的NEW_DRIB_MOTOR/NEW_DIRB_MOTOR拼写不一致；在预处理条件中未定义前者会按0参与比较，影响方向分支。没有证据证明它造成此次完全不转，也未擅自修正方向或车型参数；先确认机械表现，若反转再核对实物方向与宏定义。
- 已请求修复后g_dribbler_diag的run_requests/rejected_requests/nonzero_outputs/no_fg_stops/fg_edges/last_nonzero_ccr/disabled及电机表现，用现有观测区分命令拒绝、实际非零CCR历史、FG超时或已有运动反馈。下一步测量应针对分支，不取消保护或增加强制输出。本轮未修改固件、构建或烧录，尚未定位剩余根因。
## 47. 2026-10-19 去连锡后的吸球诊断截图：仍无FG并触发停机

- 用户新截图：requests=2、run_requests=1、rejected_requests=0、updates=0x73、no_fg_stops=1、nonzero_outputs=0xA、fg_edges=0；last_request_ms=0x1C8A、last_nonzero_ms=0x184B。当前command/target_rpm/rpm/duty/stalled/disabled均0，CCR=0、ARR=0x1A3F、CR1=0x81、CCER=3；最后非零CCR=0xBE2、CR1=0x81、CCER=3。
- 与当前d17910f实现一致：命令未被电源禁用拒绝，累计10次非零CCR写入，最后3042/6720约45.27%低有效输出；FG无上升沿，触发一次1秒无反馈停机，之后自检零命令清除stalled。ARPE启用可见，不能把非零CCR记录等同于A4931脚上已实测正常PWM或功率桥已驱动。
- 无FG既可能是电机未转的结果，也可能是转动后反馈链路异常，截图不能单独区分。下一步先确认完全不转/抖动/能转；若完全不转，在吸球命令开始后首1秒检查功率板BRK5电平（源码要求高）与驱动端PWM5实际波形，避免仅在超时后测量。先前PWM与BRK短接使相关引脚电气状态值得核对，但尚无损伤确认，不据此直接更换MCU或驱动器。
- 本轮仅记录截图和分析，未修改控制代码、CubeMX配置或无反馈保护，未构建或烧录。
## 48. 2026-10-19 电机完全不转，BRK实测3.3V

- 用户确认吸球电机完全不转，BRK测得3.3V；结合第47节无FG记录，当前应优先检查驱动启动条件，不将无FG直接归因为测速软件故障。该读数支持测点当时为释放刹车高电平，但测点是否在驱动芯片侧、是否覆盖首1秒运行窗口仍未明确。
- 后续优先确认功率板驱动端PWM5在首1秒内实际幅度/周期及吸球支路保险丝后电源电压，并确认RN4是否曾拆除、现已恢复。RN4包含霍尔负端参考下拉，整颗缺失可能使换相条件异常（见第43节），但用户尚未报告实际拆除，不把它当成既成事实或根因。
- 基线仍d17910f，本轮仅补充诊断记录，未修改固件、保护或CubeMX配置，未构建或烧录；剩余不转原因未确定。
## 49. 2026-10-19 支路22.5V、驱动端PWM正常、RN4未拆：检查驱动内部电源

- 用户逐项确认RN4从未拆除，吸球支路保险丝后电压22.5V，功率板驱动端存在此前要求的正常PWM。结合BRK=3.3V且电机完全不转，目前不再将RN4缺失、支路无供电或PB9到驱动端无PWM列为首要假设，仍不能据此确认驱动芯片内部及功率桥正常。
- 重新核对Allegro A4931 Rev9官方手册：HBIAS第8脚典型7.5V；VCP第12脚典型约VBB+7.5V，其欠压会禁用驱动。ENABLE连续高超过约3ms进入待机并关闭HBIAS。因此自检1秒无FG停机后，HBIAS下降可能是正常待机结果，不能在该时刻以低值判断芯片损坏；下一步应以示波器捕获吸球PWM开始后首1秒内HBIAS及VCP。
- 建议测量均相对功率板GND：HBIAS预期约7.5V，VCP在VBB=22.5V时典型约30V，最终应比较VCP-VBB而非要求精确30V；若内部供电正常再检查三路霍尔及公共负端参考（图纸R24/RN4分压设计约2.53V）。不让用户在可能自动启动的阶段手转电机，也不把待机时FG不跳变当作霍尔坏的证据。
- 来源：https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf ，第4、5、8至10页；核对已有原理图rn4-reference-detail.png的U9引脚及R24/RN4接线。本轮仅记录，未改代码/配置或构建烧录，基线仍d17910f。
## 50. 2026-10-19 新发现保险丝重复熔断：停止带电试验，先查过流支路

- 用户此前报告HBIAS及VCP均0V；随后发现保险丝已断，更换新件后再次熔断。尚未明确保险丝位号/额定值、再次熔断时是否连接电机，以及发生于上电还是吸球PWM开始时；不能直接认定具体MOSFET、A4931或电机损坏。
- 先前第49节支路22.5V是当时测量；新熔断状态下需重新审视供电，不能沿用它认定U9当前已正常供电。若此次熔断切断U9 VBB，HBIAS/VCP为0可由失电解释，不足以证明芯片内部电源故障。先前仅凭两个0V推断驱动内部电源异常应明确以芯片VBB有电且PWM有效为前提。
- 暂停更换保险丝后直接接6S反复上电，不加大额定值或短接保险丝，不继续自检带电测量。先断开电池、USB及调试器并确认电容放电，再拔下故障支路电机（包含霍尔线）；以保险丝负载侧对GND的断电阻值及与正常支路对比定位静态短路，区分电容充电读数与持续低阻。未发现静态短路也不能排除驱动时直通/换相异常，不以电阻正常批准直接上电。
- 已询问是否吸球支路保险丝及位号、再次熔断时电机是否插着；这些决定先查功率板静态故障还是电机/线束及动态驱动。此前另一接口功率MOS击穿的历史不直接作为本次已确认根因。本轮仅更新记录，固件/配置/保护未改，未构建或烧录，基线d17910f。
## 51. 2026-10-19 用户确认逆变器MOS击穿，更换后抖动冒烟

- 用户实物排查确认三相逆变器MOS击穿，更换后保险丝不再熔断，但电机极轻微抖动并可见冒烟，手拨无机械卡顿。尚不明确冒烟来自电机绕组、MOS/驱动板还是连接器，亦未提供替换MOS型号、损坏相/上下桥位置、限流设置或电流波形。不能把保险丝不熔断作为故障排除，也不能由手拨顺畅排除绕组短路、霍尔/相序或电气堵转。
- 即刻停止接6S自检与带电手拨，断开电池/USB/调试器后再检查。当前已可见热损伤，后续不应仅减占空比后重复通电；应先定位冒烟点、断电复查功率桥D-S/G-S及焊接/料号/引脚，并与正常通道比较，电机三对线间低阻应在相同测法下接近、相线对外壳不应导通；万用表对称不排除匝间短路。
- 核对当前cfg.h/dribbler_control.c：100ms速度更新、占空比上升限制0.5/s（正常周期约每次5个百分点）、上限55%，历史截图末次约45.3%；连续1s无FG停机。软件没有实测吸球相电流闭环，这些措施并非硬电流限制；若错误换相导致不转，无FG期间输出仍会逐步增加。故承认电流过大可能发生，但未证明是健康驱动的正常启动浪涌，更不能用已有斜坡否认过流。
- 待定位的主要分支：霍尔/相序或缺相导致电气堵转；MOS击穿后栅极驱动/栅阻/配对器件的连带损伤或更换不匹配导致异常导通；电机绕组/线束电气损伤。A4931原生有霍尔换相及采样电阻限流，但其存在不证明实装检测链正常，亦不替代损坏功率桥保护。来源为Allegro A4931 Rev9官方数据手册（已复核）。
- 首要追问冒烟位置及更换MOS的原/新型号和数量；完成断电检查后再由有限流的台架排查霍尔与栅极波形，不建议继续以6S电池试错或把普通示波器地夹接到高边源极。本轮仅记录分析，未改固件/配置/保护，未构建烧录，源码基线d17910f。
## 52. 2026-10-19 区分A4931外部PWM、硬件限流与转矩控制；烟来自MOS区

- 用户确认冒烟来自MOS区域，三颗功率器件均已更换同型号新件。按当前原理图吸球级Q4/Q5/Q6是AON6884双N-MOS封装，三封装构成六个开关；同型号替换降低型号误配疑点，但不能排除栅极驱动连带损伤、栅阻/焊接/电流检测链问题或再次损坏。MOS区冒烟不能据此认定A4931一定坏，也不能断定具体直通或某相故障。
- 复核Allegro A4931 Rev9第9页：ENABLE外部PWM低电平使能所选上下桥对，高电平进入电流衰减，控制驱动时间而非数值化转速/电流设定；实际电流受母线、转速反电动势、绕组及负载影响。芯片内部固定关断时间斩波阈值ITRIP=0.200V/RSENSE，与外部占空比不是同一个调节量。
- 重新查看原理图第2页现有渲染图：吸球采样R27=39mΩ（不是四轮27mΩ），R26=0Ω位于SENSE连接，理论斩波峰值约5.13A，仅为图纸设定、实装未测；不是5.13A平均值，不是对当前电机额定值的认可，也不是MOS桥故障的万能保护。后续优先断电核对R27是否确为39mΩ且未短接、R26与SENSE连接、栅极路径及焊接；普通万用表不足以精确确认39mΩ，应看标识/BOM并采用合适低阻测量。
- 当前固件以FG转速误差输出PWM，没有吸球相电流反馈闭环。若产品目标为转矩而非恒速，可取消恒速外环并设计电流闭环（正确换相下转矩近似与有效转矩电流成比例），但直接固定PWM不等价恒转矩；总线INA226也不直接等于吸球转矩电流。建议电流/转矩目标结合速度上限、堵转/无反馈保护及启动限制，FG可用于监控，A4931用于换相的三路Hall仍必须正常。
- 先继续保持断电，定位MOS冒烟及驱动/检测链异常，再决定控制架构；未获修改控制目标实现的明确指令，本轮未删除速度环、改采样电阻、PWM或保护，未构建烧录。基线d17910f。官方来源：https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf 。
## 53. 2026-10-19 用户检查硬件未发现异常，复核PI启动及斜坡积分

- 用户报告目前检查硬件均正常，记录为用户检查结果；尚无换相、上下桥VGS、相电流动态数据，不能据此证明运行时直通/缺相等已排除，也不无视该反馈重复要求已完成的静态检查。
- 当前d17910f源码复核：目标5000rpm且反馈0时，比例项0.15；正常100ms周期积分每次增加0.03。最终输出受0.5/s上升斜坡限制，正常每次最多增0.05。积分的条件判断只考虑0至55%幅度饱和，未考虑最终斜坡限幅，因此启动限速期间积分仍增长（隐藏积分累积），可加重后续输出增长；这是代码层面的具体限制，不代表已证明导致MOS冒烟。
- 速度控制仅在无FG持续1000ms后锁存停机；此前45.3%输出历史说明故障期间给过显著输出，不能认定为实物已标定的安全启动。后续软件方向应包含独立受限启动阶段、转速目标爬升、斜坡一致的抗积分饱和及可靠反馈后的闭环切入，保留故障锁存；安全输出/时限需由电机额定数据与限流实测确定，不能随意将5%或10%称作安全值。
- 已询问能否使用可调限流电源代替6S进行后续台架定位。鉴于实际MOS区域曾冒烟，禁止把仅降低PI或PWM作为继续无电流限制上电的依据。当前无明确改变固件行为的指令，本轮仅源码分析和记录，未改PI/配置/模式，未构建烧录。
## 54. 2026-10-19 吸球电机手册归档：maxon ECXSP16L 18V

### 来源与适用边界

- 用户提供并指定为本机吸球电机手册：E:/SRC/Version3.0/吸球电机.pdf，共8页；maxon配置产品编号B80071E6080C，修订1，文件日期05.12.2023。型号全文ECXSP16L BL KL A STD 18V；第8页关联产品标识ECXZ16L1VF35N1IL1D450A。
- 本次已提取全部8页文字，渲染页面并目视核对第3页接线、第6页配置工作点、第7至8页规格。手册是用户提供的配置资料，不把其中商业条款、网页/CAD打开说明作为执行指令，也未下载嵌入CAD或访问配置网页。
- 当前源码基线d17910f3fafc7b3340ac12fb01c70f20e1089832，尊重现有AGENT.md修改。下表为手册参数，不是现场测量；电机实物线序、损伤状态和所接负载仍需与该配置核对。

### 电气、转矩与热参数（第7至8页）

| 项目 | 手册值 |
| --- | --- |
| 类型 | 三相无刷，带霍尔传感器，1极对 |
| 额定电压 | 18 V |
| 标准功率 | 40 W |
| 空载转速 / 空载电流 | 46300 rpm / 295 mA |
| 额定转速 | 43100 rpm |
| 额定转矩（最大连续转矩） | 9.09 mNm |
| 额定电流（最大连续负载电流） | 2.73 A |
| 堵转转矩 / 堵转电流 | 149 mNm / 40.4 A |
| 相间电阻 | 0.446 Ω |
| 相间电感 | 0.0246 mH（24.6 µH） |
| 转矩常数 | 3.68 mNm/A |
| 转速常数 | 2590 rpm/V |
| 转速/转矩斜率 | 314 rpm/mNm |
| 机械时间常数 | 3.27 ms |
| 转子惯量 | 0.997 gcm² |
| 最大效率 | 83.958% |
| 最大允许转速 | 50000 rpm |
| 绕组热时间常数 / 电机热时间常数 | 4.63 s / 471 s |
| 外壳-环境热阻 / 绕组-外壳热阻 | 13.5 K/W / 2.34 K/W |
| 环境温度 / 绕组最高允许温度 | -20至100 ℃ / 155 ℃ |

- 第6页为配置器所选工作点，不能当作机器人实际吸球要求：34480 rpm、7.27 mNm，所需电压14.45 V、电流2.25 A。该页所选18V条件下列最大转速43741 rpm，与第7页额定电压下空载46300 rpm是不同栏目，分别保留，不擅自统一或替换。
- 堵转40.4A是额定电压条件下的堵转参数，不是允许持续或建议测试电流。2.73A是连续负载额定值，不能直接当作台架首次上电限流设定，也不能忽略散热条件。

### 线色与接线（第3页）

| 电缆 | 线色 | 手册定义 |
| --- | --- | --- |
| 电机AWG22 | 红 | 电机1 |
| 电机AWG22 | 黑 | 电机2 |
| 电机AWG22 | 白 | 电机3 |
| 霍尔AWG26 | 橙 | Vhall，允许3至24 V |
| 霍尔AWG26 | 蓝 | GND |
| 霍尔AWG26 | 黄 | 霍尔传感器1 |
| 霍尔AWG26 | 棕 | 霍尔传感器2 |
| 霍尔AWG26 | 灰 | 霍尔传感器3 |

- 重要：电机粗黑线是相线，不是GND；霍尔蓝线才是其GND。霍尔橙线3至24V供电范围不能套用到三相绕组额定电压，也不能套用到A4931的霍尔输入耐压。
- 手册仅给电机1/2/3及霍尔1/2/3命名，没有在所读页提供对A4931六步表的完整相位对应；不得直接假定电机1/2/3必然对应板上A/B/C且霍尔编号天然匹配。需核对实际线束及换相关系，不能带电随机换线试错。

### 对当前SRC与故障分析的影响

- cfg.h当前DRIBBLER_POLE_PAIRS=1与手册1极对一致。当前目标5000rpm是机器人固件设置，不是手册额定转速；无需为了匹配额定转速把它改为43100rpm。
- 当前6S母线实测22.5V、满充可达25.2V，均高于电机18V额定值。PWM驱动18V电机并非由额定值一项即可判定不可行，但电压、电流、转速及热限制必须设计验证；不能把PWM平均电压低于18V等同于电流安全或宣称6S已适配。
- 依据手册参数推算相间L/R约55.2µs，而现有25kHz PWM周期40µs，说明电流纹波和启动脉冲电流必须关注。该比值仅用于量级判断，不是实际相电流波形或根因证明；零速22.5/0.446约50.4A也只是忽略驱动、限流、电感及供电阻抗的直流估计，不能声称实测已到该值。
- 图纸R27=39mΩ对应A4931约5.13A斩波阈值（见第52节），而电机最大连续负载电流2.73A。二者分别涉及斩波峰值与手册额定电流，不能简单等同；但明确不能把现有限流阈值当作本电机连续运行的已验证保护。该阈值及无FG等待1秒都需要结合实际电流、换相和热状态核验。
- 用户已确认有可调限流电源（补记上一轮回复）。应继续以限流台架、独立吸球输出与电流/FG记录定位；母线电源限流不等于绕组峰值限流，输入电容也可供给瞬态电流。不能仅凭手册参数或硬件静态检查正常，判定MOS区冒烟已消除或允许继续6S自检。
- 本次授权是读取手册并同步文档。未修改PI、目标转速、PWM、极对数、采样电阻、CubeMX或测试固件；未构建、烧录或实机测试。下一步受限启动/斜坡抗积分饱和方案仍为待实现建议，MOS冒烟根因仍未确定。
## 55. 2026-10-19 修复吸球PI在PWM斜坡和无FG情况下的积分累积

### 授权、基线与范围

- 用户明确要求修正此前指出的固件问题。本轮基于d17910f3fafc7b3340ac12fb01c70f20e1089832，保留既有AGENT.md记录；实际修改Core/Src/dribbler_control.c及MDK-ARM/tests/test_board_migration.py。
- 修复第53节确认的积分处理问题，并增加无FG采样窗口禁止正向积分的约束。当前仍为速度PI，不声称已实现转矩/电流闭环、独立测试模式或完整启动状态机；第53/54节中的那些方案仍为后续建议。

### 实际行为变化

- 先计算本次可输出上限upper=min(DRIBBLER_MAX_DUTY,上一输出+斜坡步长)，积分条件同时考虑这个动态上限及零下限；被PWM爬升限制截住时，不继续沿受限方向增长积分，仍允许向可用范围退积分。
- 当前100ms采样窗口没有FG上升沿时，不允许积分增加；有反馈且输出未受限时正常积分。该条件是反馈计数约束，单个毛刺也可能计入，不代表已验证霍尔换相或电流安全。
- 从停止状态启动、全过程无FG的情况下，当前参数使输出按现有斜坡升到比例项约15%（0.00003*5000），不再靠积分继续升到历史约45.3%。它不是全局15%限幅：有反馈时仍能按原55%上限工作；如果运行前已有积分后丢FG，本修复冻结正向积分而非立即清空，既有无反馈超时仍会停机。
- 保留目标5000rpm、KP/KI、100ms采样、0.5/s爬升、55%运行上限、1s无FG超时及锁存规则。电源禁用/显式停止仍清积分和PWM；未修改四轮启动保护、音乐、GPIO/TIM/CubeMX、车型/方向宏或硬件采样电阻。未以未经标定的新占空比/时间替代已有参数。

### 回归证据与构建

- 修改生产代码前，新增drib_no_fg_integral用例在旧实现失败（无FG时输出超出纯比例项）；独立运行drib_ramp_integral亦失败（已有FG但斜坡受限后，误差回零仍留积分输出）。两种现象均先复现再修复。
- 修改后tests/test_board_migration.py在4S/6S下全部18个用例各通过，共36次，包括新增两项、FG负载响应、故障锁存、零命令恢复、正常积分、诊断历史和板级电源/停机保护。tests/test_motor_startup.py、tests/test_startup_music.py及git diff --check通过。
- Keil正常工程增量构建通过，0 Error(s)、0 Warning(s)，Code54290，RO-data5002，RW-data460，ZI-data29924；日志MDK-ARM/tmp/dribbler_integral_fix_build.log。生成MDK-ARM/SRC_Firmware/SRC_Firmware.axf及SRC_Firmware.hex。未烧录、未提交Git，未进行实机或电流/栅极波形测试。

### 限制与后续实测

- 用户已有可调限流电源，且此前MOS区出现过烟。此次是明确的软件缺陷修复，不能据此宣称MOS冒烟根因已证明、功率桥已恢复正常，或15%输出/1s等待已对该低电感电机安全。PWM限幅及反积分饱和都不等同相电流限制。
- 烧录后验证应在故障器件/焊接复核、受控供电和电流测量条件下开展，不能直接以6S电池反复自检试错。继续以实际PWM、电流、FG及换相波形区分剩余硬件动态异常和参数标定问题。
## 56. 2026-10-19 按用户要求将吸球占空比上限改为50%

- 基线d17910f及第55节未提交修复。用户明确要求最大占空比限制0.5；Core/Inc/cfg.h的DRIBBLER_MAX_DUTY由0.55f改为0.50f，吸球PI积分上限、动态输出上限均读取此宏，最终控制输出不超过50%。当前ARR=6719时上限CCR=3360。
- 本节替代第55节中关于当前55%运行上限的说明；第55节作为当时修复基线保留。保留无FG禁止积分增长、斜坡抗积分饱和、目标转速、增益、采样和故障保护。本轮未修改CubeMX或其它电机参数，未更改底层原始PWM写入接口语义，也未增加控制模式。
- tests/test_board_migration.py在4S/6S下全部36次用例通过。50%是占空比限制，不是相电流限制，不能据此认定MOS冒烟故障已解决。未烧录或实车验证。
- 本节Keil构建完成：0 Error(s)、0 Warning(s)，Code54278；日志MDK-ARM/tmp/dribbler_duty_50_build.log，正常SRC_Firmware.axf/hex已更新。git diff --check通过，未烧录。
## 57. 2026-10-19 按用户要求取消吸球速度闭环，立即输出固定30%PWM

### 当前控制行为（替代第55节PI及无FG保护方案）

- 用户明确要求取消吸球闭环：收到吸球指令直接输出0.3占空比，初始化等操作与其它电机一致。本轮基线d17910f及此前未提交工作树，未覆盖用户其它改动。
- cfg.h新增DRIBBLER_FIXED_DUTY=0.30f，保留DRIBBLER_MAX_DUTY=0.50f；删除不再使用的目标转速、KP/KI、爬升和无FG超时宏，保留1极对及100ms测速窗口。
- dribbler_set_target沿用公开接口名称。合法非零命令±1/±2/±3均立即写入30%PWM，正负号仍沿用之前的方向映射，0或越界命令立即停止。当前ARR=6719，CCR=2016，低有效驱动时间约12µs/40µs周期；预装载在下一次定时器更新生效，不再等待100ms任务或进行爬升。
- 删除速度PI、积分及斜坡、1秒无FG锁存停机。FG仍计数并发布RPM，但无FG或不同RPM不改变输出；这也意味着开环堵转不再由该FG超时停止，不能把30%当成相电流限制。板级quiet/关机/电源禁用以及正常无线通信超时停止链路保留，禁用会清除命令，解除禁用不会自行恢复旧输出。
- 保留诊断结构兼容：target_rpm/stalled/no_fg_stops在本模式为0；nonzero_outputs变为非零命令实际写PWM的次数，不再是速度任务更新次数；历史CCR和时间仍保留。action.c注释同步为固定PWM语义。

### 启动顺序及CubeMX边界

- MX_TIM11_Init不再启动PWM或立即调用HAL_TIM_MspPostInit，改为PB9推挽GPIO高，逻辑0输出。init_dribbler保持BRK5高、DIR5低、CCR0，不启动定时器。
- 新增start_dribbler，在RobotTask音乐/IMU检查结束、start_motor之后调用；保存PRIMASK并检查电源禁用，先BRK高/CCR0、UPDATE装载、清标志、检查PWM启动成功，再切PB9到TIM11 AF3。任何HAL失败仍留PB9为GPIO高，不接入AF；中断屏蔽状态正常返回时恢复。TIM11频率/极性/预装载/引脚及四轮启动不变。
- 自动审批拒绝了首次包含.ioc写入的命令，理由是用户此前要求CubeMX配置由其自行修改；该命令未执行。随后仅执行获授权的运行时代码修改，不改.ioc和自动生成的gpio.c默认电平，也未间接改写这些设置。PE8/BRK5当前GPIO默认仍低，运行时init_dribbler置高；要使GPIO初始化阶段也与四轮默认释放刹车完全一致，需用户在CubeMX中把PE8初始输出设为High/SET。本项不是已完成的配置修改。
- CubeMX重新生成会恢复MX_TIM11_Init尾部自动HAL_TIM_MspPostInit(&htim11)，须像TIM1一样移除，保留USER区域GPIO高和start_dribbler中的延后AF，否则启动顺序回归；新增test_dribbler_startup.py会检测该问题。

### 验证与工具同步

- tests/test_board_migration.py以开环行为替换已作废PI测试：立即写CCR2016、六个档位/方向、零/越界停止、FG为5000/24000rpm及连续3秒无FG均保持30%、诊断历史、电源禁用拒绝和清零；4S/6S各17项通过，共34次。
- 新增tests/test_dribbler_startup.py提取实际TIM11 GPIO/AF代码和init_dribbler/start_dribbler函数，验证零预装载后接AF、禁用不启动、PRIMASK保持、UPDATE/PWM失败不接AF；通过。初次测试桩IRQ变量与被测局部同名发生遮蔽，已改正测试桩后通过，生产代码没有该问题。
- test_motor_startup.py、test_startup_music.py及git diff --check通过。独立PB9 USB测试生成器补入显式零预装载/PWM启动/AF切换步骤，已重新生成独立工程；其50%连续测试输出仍仅用于整束功率板排线断开时，不是本次机器人30%开环输出。PB9_USB_TEST.md说明同步。
- 此前MOS区域冒烟的根因与电流/换相动态仍未实机确认。当前只按用户要求改变控制方式，不声称硬件风险已消除；未烧录、未运行实机、未提交Git。- 正常Keil工程构建完成：0 Error(s)、0 Warning(s)，Code54046、RO-data5006、RW-data452、ZI-data29924；日志MDK-ARM/tmp/dribbler_fixed_30_build.log，SRC_Firmware/SRC_Firmware.axf及hex已更新。该HEX是固定30%版本，区别于output/pb9_usb_test的隔离台架镜像。
- 独立PB9 USB工程同步构建完成：0 Error(s)、0 Warning(s)，Code7042，RO-data466，RW-data292，ZI-data11436；日志MDK-ARM/tmp/pb9_usb_test/pb9_usb_test_open_loop_sync_build.log，output/pb9_usb_test/PB9_USB_Test.axf/hex已更新。首次相对路径构建未产生目标日志并停留，已仅结束该次工具启动的进程，使用绝对工程和日志路径重新构建成功；未关闭用户原Keil进程。后续命令行构建嵌套工程应使用绝对路径。
## 58. 2026-10-19 30%开环固件烧录后，插电机的U10吸球支路再次熔断

- 用户明确确认已烧录第57节30%开环版本，本次烧断的是U10吸球支路保险丝，熔断时电机已插着。前述更换Q4/Q5/Q6后不熔断的暂时状态已被新事实取代；不能将固定PWM改动或用户静态检查正常视为故障排除。
- 固定30%是直接驱动时间控制，既不保证电机电流，也没有FG堵转超时；相较此前从停止启动且无FG时的约15%比例项上限及1s停止，当前版本可能向不转电机施加更高、持续更久的驱动。尚无电流/桥臂/霍尔波形，不判定本次由PI、正常启动浪涌或具体元件单独造成。
- 暂停接电池/USB/调试器，不再更换保险丝通电或将固定PWM继续调低试错。断电、确认电容放电、拔下电机及霍尔线后，优先重新检查新换MOS的D-S/G-S与正常支路比较，板上并联路径会影响测量，持续异常低阻需隔离确认。继而核对R27=39mΩ、SENSE连接与线束；不让用户拔电机后再次带电验证是否熔断，也不以减小PWM代替定位。
- 本轮首次只读命令因自动审批服务模型容量不足失败而未执行（不是风险判定）；本记录为按用户既有授权重试文档记录。不修改控制行为/配置、不构建或烧录；30%固件的MOS区冒烟和重复过流根因仍未确定。
## 59. 2026-10-19 对比用户指定SRC-tsak新吸球电机实现，确认方向宏拼写问题

### 实物补充与来源

- 用户确认本次U10熔断后复查三颗MOS均未击穿，R27实装为39mΩ。记录为用户检查结果，不能沿用“已再次击穿”推断，也不能由静态结果认定动态驱动无异常。
- 用户指定参考工程E:/myCode/STM32/SRC-tsak/SRC-tsak，并再次明确CubeMX如需改动应告知、不得代改。前一轮只读审批超时未执行，用户要求重新发起并手动审批后，本轮成功读取该工程Core/Src/action.c、misc.c、tim.c、gpio.c、main.c、stm32f4xx_it.c、robot.c及Core/Inc/cfg.h、gpio.h、main.h的相关内容。
- 参考目录不是Git仓库（git rev-parse返回not a git repository），没有可靠提交号；只将本次实际读取的本地源码作为对比基线，不当作TIGERs固件、硬件认证或已实测正常版本。当前主工程仍d17910f3fafc7b3340ac12fb01c70f20e1089832及第57节未提交开环工作树。

### R27的意义

- R27为低侧电流采样电阻，39mR=39mΩ=0.039Ω，电流产生Vsense=I*R27；A4931 SENSE达到典型0.2V后触发固定关断时间斩波。典型ITRIP=0.2/0.039约5.13A。
- 这不是恒定电流命令、平均电流保证、软件转矩闭环或电机可连续承受电流；用户手册最大连续负载电流2.73A仍需另行核对热及控制条件。数值正确也不能证明采样布线、栅极驱动及换相正常。MOS未击穿不等于工作时损耗/电流正常。
- 电阻增大使斩波阈值降低，减小则升高；本轮不建议盲目替换阻值或加大保险丝。官方依据仍Allegro A4931 Rev9第9页Current Regulation（https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf）。

### 参考工程实际新电机路径与当前差异

- 参考cfg.h定义的是NEW_DIRB_MOTOR=1（不是NEW_DRIB_MOTOR），当前ROBOT_VERSION=NEUMANN_S02，因而DIRB_MOTOR_VERSION=NEW_DIRB_MOTOR。action.c中do_dribbler对新电机的正命令设dir=1、负命令dir=0，gain=0.55；NEW_DRIBBLE_MOTOR_DIR=1使DIR引脚与dir同向。档位1/2/3都写MOTOR_PWM_PERIOD*0.55，即6720*0.55=3696，55%低有效驱动；无FG速度闭环，也没有输出爬升。不要将这55%覆盖用户当前明确要求的30%及50%上限。

| 项目 | SRC-tsak参考 | 当前SRC工作树 |
| --- | --- | --- |
| PWM引脚 | PB9/TIM11_CH1/AF3 | 相同 |
| TIM11计数 | 时钟168MHz、PSC0、ARR6719 | 相同 |
| PWM模式/极性 | PWM1，Low，Pulse0 | 相同 |
| 引脚驱动 | AF推挽、NoPull、Low速度 | 相同 |
| 有效运行占空比 | 新电机档位1/2/3为55%，CCR3696 | 用户指定30%，CCR2016，上限50% |
| 正命令DIR引脚 | PE10高 | 当前TT_0宏导致PE10低 |
| 运行时BRK5 | init_dribbler将PE8置高释放 | 相同 |
| 初始GPIO BRK5 | 默认低，运行初始化置高 | 相同；用户仍可CubeMX改PE8初始高 |
| ARR/CCR预装载 | 禁用/显式禁用OC1PE | 用户此前已启用/启用 |
| PWM启动 | MX_TIM11_Init中启动Base_IT及PWM后接AF，init_dribbler又使能 | GPIO高保持，音乐/IMU后零CCR装载启动再接AF |
| FG与速度环 | 参考无吸球速度闭环 | 当前FG仅遥测，无闭环或FG超时停机 |

- 检查参考HAL_TIM_PeriodElapsedCallback只有TIM14时基、TIM12任务/红外处理，没有TIM11吸球PI或电流运算；因此参考的Base_Start_IT不能当作电流闭环证据，也无需为模仿它恢复TIM11更新中断。
- 参考和当前时钟配置PLLM25、PLLN336、PLLP/2、APB2/2一致，当前已实测40µs周期亦一致；未发现PWM频率、有效极性或AF模式反向导致“30%实为70%驱动”的源码证据。

### 明确发现：当前TT_0吸球类型宏拼错，方向与参考相反

- 当前Core/Inc/cfg.h:69为#define DIRB_MOTOR_VERSION NEW_DRIB_MOTOR，但只定义了NEW_DIRB_MOTOR。当前dribbler_apply_command的#if (DIRB_MOTOR_VERSION == OLD_DIRB_MOTOR)中，未定义标识符按0参与比较，误进旧电机方向翻转分支。
- 本轮将两份cfg去掉include后用GCC真实预处理验证：CURRENT输出DIRECTION_BRANCH_OLD，REFERENCE输出DIRECTION_BRANCH_NEW。不是仅凭拼写猜测，也未执行固件。
- 对齐参考新电机定义的具体建议：将当前cfg.h TT_0分支这一行改为#define DIRB_MOTOR_VERSION NEW_DIRB_MOTOR。它是C宏定义修正，不需要CubeMX。此变更会改变正负命令的物理转向，必须明确实物吸球方向；按项目约定分析任务先记录，本轮只读比较，没有擅自改变用户固件行为。
- 方向相反可解释吸/吐球方向与参考不同，但不足以证明重复熔断或MOS区域冒烟的根因；正确匹配的霍尔六步系统应支持DIR换向。仍缺相电流、霍尔相位与VGS动态证据，不能据此继续带故障试烧。

### CubeMX结论与本轮验证边界

- 目前没有源码依据要求改变TIM11频率、极性、PB9 AF3/推挽或GPIO速度；与参考这些项目已一致。预装载差异只改变更新生效时刻，本轮未证明它导致熔断，不建议为完全复制参考而回退用户设置。若用户希望初始释放刹车，在CubeMX设置PE8/MOTOR_BRK5 GPIO output level=High/SET即可，此项是此前建议、不是本轮已修改。
- 本轮仅更新AGENT.md，未改cfg.h宏、CubeMX/ioc、GPIO/TIM、固定30%逻辑、参考工程或测试代码；未构建、烧录或实机测试。前两轮读工程自动审批超时的阻塞已通过用户本轮手动审批解除，本轮对比已完成。
## 60. 2026-10-19 迁移SRC-tsak吸球档位与方向逻辑

### 授权、基线与用户改动

- 用户在第59节对比后明确要求直接迁移SRC-tsak的drib代码，并在获知迁移将采用参考新电机55%输出后要求发起手动审批。本轮执行迁移；此前一轮两次自动审批容量失败均未执行改动，不记作已迁移。
- 当前HEAD仍d17910f及未提交工作树。迁移前发现用户已自行将类型名统一为OLD_DRIBBLE_MOTOR=0、NEW_DRIBBLE_MOTOR=1，并将TT_0分支修正为NEW_DRIBBLE_MOTOR。保留该用户命名及其车型分支变更，没有恢复参考中的旧拼写。首次脚本因预期旧宏不存在而在首个assert退出，未写文件；复查后适配用户现状执行。

### 实际迁入行为

- 从E:/myCode/STM32/SRC-tsak/SRC-tsak/Core/Src/action.c的do_dribbler/set_dribbler迁入档位、增益与方向语义；参考源码无可靠Git提交号，详见第59节。
- 当前保留do_dribbler -> dribbler_set_target的公开接口适配，以及dribbler_apply_command中的统一输出/诊断，将参考的电机类型gain提取到cfg.h：旧电机固定50%，新电机固定55%，各自1/2/3档相同，正负号选择方向，0及越界停止。CCR仍用实际ARR+1计算，在当前6720周期下与参考MOTOR_PWM_PERIOD公式一致：旧电机3360，新电机3696。
- 当前TT_0选NEW_DRIBBLE_MOTOR，NEW_DRIBBLE_MOTOR_DIR=1：正命令PE10/DIR5高，负命令低；0命令也按参考的非负分支给DIR高，同时CCR0。旧类型正/零方向低、负方向高。显式使用两个已定义的类型分支，未知类型编译报错，避免残留拼错宏参与比较。
- DRIBBLER_MAX_DUTY随参考新电机输出改为0.55f；这次经用户获知并授权的参考迁移替代第56/57节的50%上限和30%固定输出。不是将55%声明为该故障实物的安全电流设定。
- 保留set_dribbler的电源禁用检查及周期范围钳位、临界区、诊断历史、FG遥测。无速度PI、无爬升、无FG超时停机。保留原本对合法输入范围的检查，不复制参考对极端负整数取绝对值的潜在溢出。
- 不移植参考的提前PWM启动及无吸球处理意义的TIM11更新中断，保留第57节已建立的GPIO高/零输出/音乐后启动/延后AF运行时策略。这是为保留当前整机启动与电源保护的适配，不宣称逐行照搬整个工程。

### 配置边界与验证

- 未改.ioc、gpio.c、TIM11频率/极性/引脚/预装载、PB9推挽速度或其它CubeMX设置；未改参考目录、四轮、射球及音乐逻辑。PE8初始High仍是需用户自行CubeMX操作的既有建议，本轮未代改。
- tests/test_board_migration.py增加来自参考的明确预期值与方向断言，并在仅位于tmp的cfg副本中分别选择当前新类型和旧类型，各跑4S/6S的17项，共68次通过。覆盖即时输出、三档及反向、停止DIR、越界、FG不同转速和无FG持续输出、诊断与板级保护；没有修改生产车型来跑旧分支。
- tests/test_dribbler_startup.py、tests/test_motor_startup.py、tests/test_startup_music.py及git diff --check通过。此前保留的诊断/启动修改没有回退。
- 本轮未烧录、未实车测电流或运行电机、未提交Git。过去U10重复熔断和MOS区冒烟仍未证明根因；迁移和宿主测试只能确认软件行为与参考对应，不能替代受控电流、换相/栅极波形验证。
- 本节正常Keil工程构建完成：0 Error(s)、0 Warning(s)，Code54038、RO-data5006、RW-data452、ZI-data29924；日志MDK-ARM/tmp/dribbler_reference_port_build.log，SRC_Firmware/SRC_Firmware.axf及SRC_Firmware.hex已更新。本次HEX选用用户当前TT_0/NEW_DRIBBLE_MOTOR分支，固定55%输出，未烧录。
## 61. 2026-10-19 澄清设置占空比与示波器高电平占空比的互补关系

- 用户反馈实际测量占空比为1减设置值，质疑迁移极性。重新读取当前tim.c、dribbler_control.c、action.c及cfg.h：TIM11为PWM1、TIM_OCPOLARITY_LOW，CCR=floor(duty*(ARR+1))，写入过程中没有再取反。参考SRC-tsak同样PWM1/LOW。
- 当前参数duty定义为A4931 ENABLE低电平驱动使能指令的时间比例。因低有效，Dlow=CCR/(ARR+1)，Dhigh=1-Dlow；不是通常示波器+Duty（高电平占空比）的定义。之前笼统说“55%PWM”未明确极性，容易产生相反理解，现统一用“55%低电平有效占空比”表述。
- 当前新电机duty=0.55、ARR6719、CCR3696：周期40µs，PB9低约22µs、高约18µs；示波器+Duty约45%，-Duty约55%。此前duty0.30应低30%/高70%；停止CCR0应恒高（低有效使能0%，理想高比例100%，无边沿时仪器可能不显示占空比）。
- A4931官方Enable Logic明确ENB低使能、高切断对应驱动进入电流衰减。因此高电平测量为1-duty与当前及参考实现一致，不应为了让+Duty数值等于软件duty而擅自翻转极性或再计算ARR-CCR。该“使能指令比例”不保证实际桥臂导通时间、电流或转矩，因为内部限流、死区和故障状态仍可改变桥臂行为。
- 只凭用户当前描述尚未取得标注+Duty/-Duty的完整波形；若用户测量的低电平比例才是1-duty，则与上述预期不符，需要核对测点、运行寄存器与烧录镜像，不能预先认定完全正常。该关系也不证明过去熔断/冒烟原因已消除。
- 官方来源复核：https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf ，第9页Enable Logic。本轮仅更新说明，未修改固件占空比/极性/CubeMX，未构建或烧录；基线d17910f及第60节迁移工作树。
## 62. 2026-10-19 用户将输出上限降至15%，6S直供仍熔断

- 用户澄清本次是保险丝熔断，不是已确认MOS再次击穿；仍沿用第59节用户检查MOS未击穿的证据边界。用户回复占空比15%，在cfg.h设置，频率25kHz，供电为6S电池直接连接，并未使用其已有的可调限流电源。
- 当前源码实读：新类型DRIBBLER_FIXED_DUTY仍0.55f，但用户已将DRIBBLER_MAX_DUTY改成0.15f；dribbler_apply_command确实先将duty钳至上限再乘ARR+1，因而这项改动会生效，不是只改了未选中的旧电机分支。未取得本次运行CCR及低脉宽实测，不能将源码等同于已确认烧录镜像和实物波形。
- 在当前PWM1/Low、ARR6719下，预期CCR1008，低有效指令15%，高电平85%，周期40µs、低脉宽6µs。此前第60节55%新类型默认值仍存在，但本次有效上限由用户15%覆盖；不覆盖或恢复用户改动。
- 依据手册相间R0.446Ω、L24.6µH，L/R约55.2µs。仅做零速、单个脉冲、初始电流0、理想RL且忽略内部限流的量级估算，i(t)=V/R*(1-exp(-tR/L))：6µs在22.5V约5.20A，25.2V约5.82A。这不是测得的电流，也不是周期平均/RMS或保险丝电流，不能以峰值与保险丝额定值直接比较判定熔断。
- 15%PWM不等于15%电流，每个有效脉冲仍施加母线电压。当前已无速度PI，所以本次没有PI积分增长机制。重复熔断根因仍可能涉及实际输出/烧录与测点、换相/相序或缺相、运行时栅极驱动异常/交叠导通、采样限流链异常等，静态MOS未短路和R27标值正确不足以排除这些动态问题，不宣称已确定某一故障。
- 暂停6S反复试烧；后续必须在受控限流条件下查实际电流、VGS、霍尔换相和SENSE斩波响应，不能继续仅减duty。尚未取得替换U10的实际额定电流/型号及熔断瞬态，不将图纸器件额定值当作用户实装值。
- 本轮仅记录分析，未改控制代码、CubeMX、用户0.15上限，未构建/烧录。继续以A4931官方Rev9的Enable Logic、Current Regulation、PWM Blank Timer和Charge Pump作为器件行为参考，源https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf 。
## 63. 2026-10-19 以曾稳定运行为故障回归主线，纠正启动电流推断的优先级

- 用户再次强调该吸球电机曾经稳定旋转，因此不应把本身设计电流作为此次异常过流的既定原因。采用这一实测时间线作为排查主线；此前第31节也记录过四轮防突动修改前、已有FG闭环时吸球正常。
- 第62节理想RL单脉冲约5.2A估算只说明低PWM不等于电流限制，不能证明本次真实相电流或保险丝电流，更不能作为“电机天然不适配/正常启动必然烧保险丝”的依据。反复强调这一量级容易错误转移对回归及修复后异常的关注，后续不再将其列为已确认根因或首要解释。
- 当前应优先定位从稳定到异常之间发生的变化：固件启动/使能/方向/实际PWM路径、接线和焊接变化、MOS/驱动维修后的运行状态等。软件继续作为回归嫌疑，不能仅凭CCR正确或静态MOS未短路就宣布软件或动态硬件已排除。
- 不基于本次讨论改电机选型、采样电阻、保险丝额定值或盲目减PWM；稳定版本的准确固件及硬件条件和故障期间的受控电流/换相证据尚未完整建立，不能在无证据下指定某次修改或某元件为唯一原因。
- 本轮仅修订分析重点并更新记录，不改变用户当前15%上限、固件/CubeMX，不构建或烧录。
## 64. 2026-10-19 逐网复核吸球原理图：器件候选、测量参考与PCB可修性

### 新事实及证据范围

- 用户再次明确六颗MOS的D-S静态测量均未击穿，询问具体可能损坏器件及PCB内层是否不可修；补充MOS区没有明显烧蚀、起泡、分层或焊盘损伤。采用该事实，不断言MOS再次D-S击穿，也不以此宣布栅极/动态驱动均正常。
- 本轮重新读取E:/SRC/Version3.0/powerboard_schematic.pdf第2页，渲染3.5至5倍局部图，逐条核对U9、Q4/Q5/Q6、RN5、电荷泵、采样及霍尔网络。此前曾稳定运行仍是排查回归与维修后异常的主线，不转而认定原始设计电流必然过大；软件仍未被实车证据排除。

### 栅极和相节点：优先检查组

- U9=A4931，Q4/Q5/Q6各为双MOS封装AON6884，共六个开关。RN5的三个实际使用支路为10Ω，分别连接U9 GHA(21)至Q4 G1、GHB(19)至Q5 G1、GHC(17)至Q6 G1；第四支路未使用。低侧GLA(16)直接到Q4 G2、GLB(15)到Q5 G2、GLC(14)到Q6 G2，不经过RN5。
- 优先候选是U9输出级/焊接、RN5开路/阻值增大或焊接、G-S/G-D泄漏及门极连接异常。这些可影响开通、关断和换相，即使D-S没有静态短路也可能在工作时产生大损耗或异常电流。不能把“最先检查”当成已确定U9坏了，更不能仅以更换过芯片排除后续连带损伤。
- 同时检查相节点回授：U9 SA(22)至Q4 S1/A_OUT5，SB(20)至Q5 S1/B_OUT5，SC(18)至Q6 S1/C_OUT5应直接连通；以及U9 GND(7)/EPAD(29)的地回路。维修后的细线、过孔、焊盘接触异常属于可定位的网路故障，不等于PCB已整体报废。
- 断电与放电后逐网测端点连续性并与正常通道比较。若要判断MOS本体G-S/G-D泄漏，应排除U9和其它并联路径；不能规定所有在板G-S都应无穷大，也不能把驱动器内部下拉造成的低读数直接判作MOS短路。

### 电荷泵、采样、霍尔及母线支路

- C33=100nF跨CP1(9)/CP2(10)，C35=100nF跨VCP(12)/VBB(11)，C28=100nF跨HBIAS(8)/GND。C33/C35损坏、开焊或U9供电/电荷泵异常值得核对；工作时HBIAS典型约7.5V、VCP-VBB典型约7.5V。A4931存在VCP/VBB欠压关断，不能把任意电荷泵异常都简单断言为MOS半导通或保险丝熔断，仍需实际动态证据。
- R26=0Ω是U9 SENSE(13)至R27上端采样节点的连接。R27实物39mΩ正确仍不足以验证该通路：R26开焊、SENSE焊盘接触异常或R27被其它路径旁路，均可能让采样失真或不可靠。图纸设定约5.13A阈值不等于已验证生效。
- 霍尔链为CN2的5V_BAT/GND、RN7的5.1k上拉、RN6三个330Ω串联支路、D11/D2/D3保护及U9 HA5/HB5/HC5。R24=10k和RN4的5.1k下拉产生公共负端参考，HBIAS正常时约2.53V。单路故障可能造成换相/缺相；A4931逻辑表对三Hall全同的无效状态关闭输出，所以不把任何“无霍尔”都直接等同于驱动短路。
- C29/C30/C31/C32各10µF直接跨U10后母线与GND，若裂纹/漏电/短路可导致与PWM无关的支路过流，MOS的D-S检查不能排除它们。用户此前拆除所有18V TVS，按D1已移除处理，不再将D1列为首要嫌疑；若实物后来恢复安装则需另行核对。元件冒烟位置目前只定位到MOS区域，不等于已识别具体冒烟元件。

### 必须避免的测量连接误判

- C35下端为VBB电池正电源，不是GND；C33两端也不是GND。C35两端电压是VCP-VBB，不能将其中任一端当作普通探头地。
- 普通共地示波器若其它通道地夹/USB/调试器已使电路GND与仪器地连接，再将地夹夹到VBB、相线或高侧源极，可能建立短路回路；单个完全浮置电池系统与有其它回路情形须区分，不声称任一非地连接必然短路。上管VGS应使用合适差分测量，不能以G对主板地替代G-S，更不能拆仪器保护地。
- 已询问用户此前HBIAS/VCP/MOS使用万用表还是示波器，以及地夹和USB接法；尚未取得回答，不能把测量短路列为已确认根因或归责用户。此前建议在电容焊盘测量未明确C35的浮置参考，现补充这一必要边界。

### PCB内部损伤与可修性

- 原理图不提供内层/叠层/实际铜箔与过孔状态。内层短路、过孔/走线开裂、局部基材碳化均可能发生，但用户观察无明显板体损伤时，无证据优先认定整板不可修。
- 只有隔离相关元件、清理并排除表面焊接和并联路径后，本应绝缘的板上网络仍异常低阻/漏电，或端点铜路仍开路，才进一步支持板级故障。已隔离时若异常随元件移走而消失，应回到器件/焊接定位，不先归因板内。
- 局部开路/过孔故障往往可评估补线修复；大电流网络内层短路、广泛碳化/分层且无法可靠隔离时，建议更换PCB而非反复带6S试烧。没有视觉烧蚀不等于排除隐裂或板内故障，也不等于MLCC完好。

### 来源及本轮操作

- 主要资料为上述用户原理图第2页；器件行为复核Allegro A4931 Rev9与AOS AON6884 Rev1.1：https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf ，https://www.aosmd.com/sites/default/files/res/datasheets/AON6884.pdf 。
- 以TI栅极驱动文章解释寄生误导通/驱动速度与损耗机制，仅用于故障模式推断，不把TI专有驱动实现套到A4931：https://www.ti.com/document-viewer/lit/html/SSZTCJ4/GUID-024462B9-956D-48E4-8899-E1B68E489607 。MLCC隐裂/绝缘劣化参考Murata：https://article.murata.com/en-sg/article/strain-crack-mechanism-and-preventive-measures-for-mlcc 。探头共地边界参考Tektronix/Keysight官方资料。
- 本轮只读原理图、生成临时局部图、分析并更新记录；不改固件、CubeMX或用户15%上限，不构建烧录。仍不能远程指定已损坏器件，先以U9/RN5及相节点、采样/电荷泵实际连接区分候选。
- 第64节后用户回复：HBIAS/VCP使用示波器，MOS使用万用表，未接调试器，并确认操作无误。按用户反馈不再把探头接法列为首要嫌疑或据此归责；没有取得具体地夹位置不等于已证明发生接地短路。现有首轮检查重点为RN5三个栅极支路、SA/SB/SC相节点回授、R26/SENSE采样连续性及U9焊接/地回路，PCB不可修仍无证据。
## 65. 2026-10-19 用户完成第64节五组静态检查

- 用户回复：RN5/栅极通路连通，SA/SB/SC到相节点连通，R26/SENSE采样连接及电阻正常，U9焊接/接地正常，霍尔链及支路电容检查正常。按用户实测记录，不再反复要求同一组连续性检查；未取得具体仪器、读数或工作电压下绝缘测试，不把这些结论扩展为动态性能已证实正常。
- 静态断线、明显虚焊和硬短路的嫌疑下降；尚不能在U9动态驱动、MOS栅极/切换行为、负载侧换相与实际控制时序之间定位。无证据指定某器件必坏，也没有新增证据支持PCB内层不可修。
- 下一步建议使用现有限流电源，先整束拔下吸球电机连接器、保持停止，观察上电充电暂态之后是否有持续异常耗流/发热；没有异常只能排除相应静态重负载，因拔下霍尔可能使A4931禁止输出，不能证明驱动动态正常。连接器只在断电后插拔，不建议改大保险丝或再接6S试烧。
- 后续若接回电机受控测试，需同时区分母线欠压/限流状态、实际ENABLE/BRK/DIR、有效霍尔变化、VGS和SENSE波形；电源进入CC导致电压塌陷时，不能以电机不转单独判断驱动故障。SENSE典型阈值约0.2V，但受消隐/斩波时序影响，不能把任意瞬时尖峰越阈视为限流失效。
- 源码基线沿用d17910f及现有未提交工作树；本轮只更新记录，无固件/CubeMX改动，无构建或烧录。官方A4931 Rev9复核：https://www.allegromicro.com/-/media/files/datasheets/a4931-datasheet.pdf 。
## 66. 2026-10-19 接电机后吸球指令触发1A电源保护

- 用户新实测：不接电机不熔断；接上电机，以可调电源限流上限1A供电，到吸球阶段立即触发保护。不能把这扩写成已测得短路电流、已确认MOS直通或电机正常启动必然超过1A。
- 1A是电源侧限制，若整机共用还含主控/四轮/升压等；不同于R27检测的桥臂回流电流或A4931约5.13A斩波阈值。电源CC降压与OCP锁断的后果不同，尚未确认是哪一种。母线塌陷也可能引起驱动欠压/MCU重启，必须区分其与初始故障先后。
- 此结果支持异常与负载接入/驱动阶段有关，但整束拔插同时改变霍尔状态，不能据此排除无负载时被禁用的桥臂动态故障。此前6S条件保险丝熔断和MOS区冒烟仍是实际异常，不能以1A可能限制正常启动而否认。
- 已询问本次设定电压、CC持续输出还是OCP切断、吸球前基线电流及四轮是否同时运转。未建议提高限流值/保险丝或恢复6S试烧；待这些条件明确后再确定动态采集方案，不反复做已通过的连续性检查。
- 基线仍d17910f及现有工作树；只更新AGENT.md，未改固件/CubeMX/15%上限，无构建或烧录。复核A4931官方Rev9的供电/欠压及采样阈值，来源同第65节。
## 67. 2026-10-19 确认22V/1A为保护切断，待机69mA，吸球先于四轮

- 用户明确：电源22V，上限1A，吸球开始时直接切断输出；吸球前0.069A，四轮尚未运行。纠正第66节四轮并行负载的未证实假设，不再要求用户确认源码可查的自检顺序。未取得电源型号及保护延迟，按用户描述记录为保护切断，不宣称已测得精确峰值。
- 本轮实读Core/Src/robot.c：SELFTEST_MODE先do_dribbler(1)持续2秒、停止，再平射/挑射，之后才四轮旋转。因此吸球触发阶段不能将行走轮作为1A保护的主要解释。待机69mA不支持持续大电流母线硬短路，仍不能排除启动时的动态异常。
- 实读当前cfg.h的MAX仍0.15f，dribbler_control.c先钳位再算CCR；TIM11为PSC0/ARR6719/PWM1/Low。源码预期25kHz、CCR1008、低6us/高34us，无速度PI累计；源码不是运行镜像/实际输入波形的替代证据。
- 1A锁断可在电机尚未产生可见转动前终止供电，单凭该现象既不能确认正常启动，也不能确认短路/换相错误；之前冒烟与熔断仍需定位。暂不建议提高限流、关闭保护或接回电池。
- 建议下一项区分性证据为同一次受控启动的PWM5/ENB与R27上端对板GND的单次波形，保持22V/1A保护，两探头地均只接板GND。核对驱动前后的低脉宽及采样电流随时间变化，不以掉电后HBIAS/VCP=0判U9坏，不把SENSE窄尖峰超过200mV直接判限流损坏（手册消隐1.5us）。R27电压仅对应其实际电流路径，需结合后续桥臂波形才能区分真实负载/错误换相/交叠导通。
- 无固件或CubeMX改动，无构建/烧录；基线d17910f及当前未提交工作树。A4931官方Rev9第9页Current Regulation/Enable Logic/PWM Blank Timer已重新核对。
## 68. 2026-10-20 暂停吸球排查，复核升压与TIGERs差异

- 基线d80b9f3501492870f11270f9beb8e1cf79a3d6bb，开始工作区干净。读取本文件历史及BOARD_MIGRATION.md，重新逐项读取当前SRC板级监控/初始化/调度/射球、TIGERs本地kicker/adc_kicker/power_control和上层模式调用；参考仍为本地v2020目录，无可靠提交号，不代表现役固件。
- 用户更新实测：BAT22.5V、CAP0.01V、PC4一直低；仪器/时间分辨率和状态快照未确认。替代第16节旧读数，不直接指定根因。
- 新增MDK-ARM/BOOST_COMPARISON.md，记录对照、修改优先级与证据限制。确认SRC本已有自动充电，无须额外AUTO_CHARGE命令；22.5V高于6S的19.8V门限。应先补禁充原因/热锁存/重试/原始ADC诊断，核实PC13信号语义，再修明确的射球后冷却门控缺口。滤波/独立过压检查需按本板额定与10ms周期适配，不能照搬200/220/260V或TIGERs引脚。
- 临时tmp/boost_review.py使用生产board_monitor.c复现：22.5V/约25°C/近零CAP正常CHG高；DONE持续低时30s重试且约10ms高脉冲；启动I2C失效5s后恢复22.5V有效数据，quiet仍锁存并CHG低。模拟CAP0.01V量化为0，不冒称硬件精确复现。若同次上电吸球/四轮仍能运行，quiet作为唯一原因嫌疑下降；若示波器覆盖完整周期无高脉冲，单独DONE低不足解释。
- 4S/6S各12项相关既有回归通过，共24项；完整板级脚本在drib_diag失败，预期55%与用户现行15%上限不一致，未改参数迎合测试。临时复现最初漏调board_monitor_init导致桩断言失败，补齐初始化后通过；不是固件SPI初始化故障。
- 仅新增分析文档与本记录、生成tmp工件；未改生产固件/CubeMX/协议/用户15%上限，未Keil构建、烧录或实车测试。需完整门控快照确定本次PC4低的具体触发源，不删保护或强行使能。

## 69. 2026-10-20 实现禁充原因OLED及射球后冷却门控

- 用户确认PC13接DONE，并明确要求禁充原因上OLED、补射球后冷却禁充。基线仍d80b9f3及第68节文档工作树，尊重用户15%吸球上限。
- 修改Core/Inc/board_monitor.h、cfg.h、Core/Src/board_monitor.c、misc.c。新增多原因位图和故障来源/重试/冷却剩余时间快照；DONE低电容异常与无进度分开诊断，保留原保护和30s重试。PC13不再作为未经确认的独立FAULT解释。
- OLED y43..50显示CHG状态，多个原因1.5s轮换，DONE/NO RISE显示重试秒数；四轮行移至10/18/26/34，无覆盖底部模式栏。关机时继续既有专页。标签说明和操作见MDK-ARM/BOOST_COMPARISON.md新增实现节。
- KICKER_CHARGE_COOLDOWN_MS=80，取本地TIGERs默认值，非本板最佳值实测。每次成功kick_pulse调用board_notify_kick记事件；监控确认TIM9停止后开始80ms冷却，捕获短于10ms监控周期的脉冲。最终输出临界区重查射球/冷却/quiet/shutdown，冷却期间CHG低；不新增普通射球频率限制，不改目标180V、引脚、CubeMX或吸球控制。
- 新tests/test_charge_inhibit.py：4S/6S各10项新场景+12项相关原有回归，共44项通过；含DONE重试、无进度、热回差、并发原因、quiet恢复、回差带、长/短脉冲冷却与32位计时回绕。test_charge_oled.py提取实际显示块验证标签/轮换/倒计时和128像素宽度约束，启动音乐和四轮启动回归通过。完整旧板级套件仍有与15%上限不匹配的吸球55%断言，未改参数迎合。
- Keil完整构建0Error/0Warning，Code54622、RO-data5118、RW-data500、ZI-data29932；MDK-ARM/tmp/charge_inhibit_build.log，正常SRC_Firmware.axf/hex更新，未烧录。git diff --check通过。宿主桩不等于实屏/实板验证，需烧录后读取CHG行定位实机未升压原因。
## 70. 2026-10-20 实机OLED显示CHG:NO RISE

- 用户反馈CHG:NO RISE。复核当前board_monitor_update：PC4读高、kicker_valid为真且进度计时达到3s会记录该原因，低电容DONE有效时优先记DONE LOW。因此本次反馈属于无电压进度分类，不能当作DONE低触发。
- 故障后主动拉低PC4并等30s，故障等待期测低与早期反馈相容；未独立测量实际高压电容，不能凭ADC有效性判定测压链准确。本轮只记录，不改固件。

## 71. 2026-10-20 逐引脚复核v13b：发现射球极性冲突及参考源码状态读取缺陷

- 用户新确认CHG确实高约3s，实物升压/射球板为TIGERs Kicking Device v13b。覆盖第68节“PC4一直低”的早期不完整观测；不再以完全无充电请求解释NO RISE。
- 基线d80b9f3加第69节未提交工作树。读取本地TIGERs main2019的main/system/SPI2/TIM13/TIM14/I2C2/设备绑定、shared的GPIO/SPI/DMA/timer_opm/adc_kicker/kicker/power_control与SRC对应源码；详细对比见MDK-ARM/BOOST_PIN_AUDIT.md（引脚配置、完整X1接口、上电/ADC/充电/射球/冷却、LT3751硬件引脚）。
- 使用PDF技能渲染并逐页查看electronics-master/v2020/PDF/Kicking Device v13b.pdf两页和SRC Mainboard_2026-10-10.pdf，再用EAGLE同名.sch网表确认：KICK-DIS1/2经SI8620非反相隔离后接FAN3229负输入，正输入12V。v13b实际控制端高关闭、低射球。
- 关键修订：TIGERs timer_opm OCM_DISABLED=101为强制有效高（CC1P0），PWM模式110/PWM1产生低脉冲，OPM回零高。SRC当前TIM9 PWM2/High、CCR=period-ticks则低等待/高脉冲/结束回零低，和v13b反相链冲突。第69节“只补80ms冷却即适配完善”的含义不能成立，冷却只控制CHG，未保证DIS关闭。此前board_kicker_ready的CC1P/2P检查和pulse宿主测试也以高有效为假设，通过不能证明实板极性正确。
- 若实际DIS低到达驱动，IGBT门极可能被拉高，放电支路可能阻止电容充电，是NO RISE的优先候选；未测DIS/门极，且初始通道禁用时真实脚电平不能由PWM公式保证，不宣称已经证明根因。建议下一实现统一两路启动/停止/脉冲/故障/关机放电为高关闭、受控低脉冲，并同步安全守卫和测试；本轮按用户详细比较要求不擅自改变固件。旧镜像不宜继续用于v13b带高压射球自检。
- CHG均高有效静态允许，TIGERs先输入弱下拉10ms/弱上拉10ms用板上R2=10k识别，再先写低后切推挽；不是特殊激活脉冲。SPI仅读ADC，未发现另一根MCU升压使能或须下发的LT3751寄存器命令；HVGATE由LT3751自己产生。X1.11的12V与X1.12主电源是硬件供电条件，不由CHG产生。
- SCK实际TIGERs72MHz/16=4.5MHz，SRC42MHz/8=5.25MHz，mode3/64字节CS低、通道选择、弃前两帧和字节序语义一致。TI ADC122S051手册3.2–8MHz覆盖两者，不应盲将SRC分频改16（会成2.625MHz）。温度/电容分压换算相同，滤波/采样周期不同。
- 本地参考自身问题：GPIOPinIsSet宏读BSRR而非ODR/IDR，kicker chargeControl用它读取isCharging。若BSRR读零会持续重置进度基准，设计中的3s/5V保护不一定实际生效；不把本地包视作完美参考或现役行为，不移植该宏。SRC HAL_GPIO_ReadPin读IDR不应改成它。
- 原理图区分：Version3.0/powerboard_schematic.pdf仅电源/电机页；POWBRD_Flyback/SRC_PowerBrd_Flyback.pdf是旧16V/VFB方案，均不代替此次确认的v13b。器件行为核对ADI LT3751、onsemi FAN3229及TI ADC122S051官方手册，链接见详细报告。RM0433在线PDF因大小限制未完整读取，BSRR结论以本地代码和寄存器用途为主，不声称实测TIGERs。
- tmp/compare_kick_polarity.py按实际模式字段推演电平，确认OPM回零后TIGERs高/SRC低；非实物波形。只更新报告/记录，不改控制源码、CubeMX、参数，不构建/烧录；本节同时修复第70节上次写入时产生的中文乱码（内容依据当时工具读取与对话恢复）。

## 72. 2026-10-20 修复v13b DIS1/DIS2低有效极性

- 用户明确要求先修DIS1/DIS2。基线d80b9f3及第69/71节未提交工作树，保留OLED禁充原因、80ms冷却和用户15%吸球上限。未更改CHG保护、SPI、电容目标或其它电机配置。
- 实际修改Core/Src/main.c、tim.c、misc.c、board_monitor.c、Core/Inc/misc.h及SRC_Firmware.ioc。TIM9仍PWM2/原PSC3359/ARR635，只将两路极性改LOW；普通射球及专用关机放电安全守卫同步要求CC1P/CC2P均为1。
- main的SysInit在MX_GPIO/其它外设前建立PA2/PA3推挽高+上拉；TIM9初始化再次保持高，MspPostInit保持GPIO高，不自动接AF。复位到该函数执行前仍依赖硬件偏置，软件不能保证未运行阶段的电平。
- kicker_stop先把两路切到GPIO高，再停两通道/共享计数器，保存恢复PRIMASK。shoot_off/chip_off均走该路径，故普通停止、故障、quiet及关机停止不依赖禁用定时器后的隐含脚电平。
- kick_pulse先保持两路GPIO高、停止旧输出、CHG低，装载原CCR=period-ticks并UPDATE；HAL启动成功后显式清CEN、再次UPDATE复位计数和预分频相位，在已使能但停止的PWM2/LOW高电平状态接入选中通道AF，再开CEN。未选通道一直GPIO高；HAL启动失败经shooter_off回退两路高。特意不用__HAL_TIM_DISABLE完成此中间停表：真实F4 HAL在CCxE仍使能时该宏不会清CEN。
- 脉冲持续时间/原力度映射不变：20us/tick，1档100us低，127档12.7ms低，仍位于636计数周期末尾；结束OPM回零高。没有顺便改变发射延迟、协议力度映射或普通射球频率策略。80ms冷却仍从监控观察到结束后计算。
- 更新tests/test_board_migration.py的GPIO/真实HAL停表限制模型、生产函数提取与极性断言；新tests/test_dis_polarity.py在4S/6S下共8个运行场景及启动/CubeMX静态检查通过，覆盖两路最短/最长低脉冲、回零高、stop高、HAL失败、IRQ状态保持、错误极性拒绝。tests/test_charge_inhibit.py的44项相关回归、OLED标签/轮换测试、音乐及四轮启动测试通过，git diff --check通过。旧完整板级套件的55%吸球预期与当前15%不一致仍未扩大修复。
- 最终Keil构建0Error/0Warning，Code54754、RO-data5114、RW-data500、ZI-data29932；日志MDK-ARM/tmp/dis_polarity_final_build.log，正常SRC_Firmware.axf/hex更新，未烧录/实板测波形。不将代码修复当成已经证实NO RISE消除。
- CubeMX再生成会把TIM9 MspPostInit恢复成AF，必须保留本轮延后AF策略；新增测试检查该风险。ioc已同步LOW极性，避免普通重生成恢复高有效。后续受控验证应先确认PA2/PA3静态高及低脉冲、结束高，再观察CAP与CHG，不能以宿主通过代替实测。

## 73. 2026-10-20 DONE方向与用户此前同时拉高的实验

- 用户新增历史实测：此前实验DONE要和CHG一起拉高才升压。保留该现象，不据此将DONE改为第二路使能。尚不知当时是GPIO推挽高、输入上拉还是电阻上拉，以及测试固件是否读取DONE后禁止CHG。
- 重新核对ADI LT3751官方手册Pin Functions第8页及DONE/FAULT Pin Design第20页：DONE(7)是开集电极指示输出，需上拉；对主控PC13而言应是输入。CHARGE(8)才是启停输入。DONE会在充满或FAULT有效时拉低，启动时也可能短暂低；其高电平表示输出管释放且上拉有效，不是主控应强制送高的第二个启动命令。手册要求DONE/FAULT电流限制1mA，不能直接推挽高对抗其拉低。
- 当前gpio.c:95–98实际PC13为GPIO_MODE_INPUT/GPIO_PULLUP，未配置推挽输出。v13b图DONE直接到X1.8并有100nF对地；结合主控上拉讨论，不宣称已测得上拉电阻、RC或实物连线状态。此前同时拉高可能涉及提供缺失上拉，或强制高掩盖当时软件DONE禁充判断；这些只是待区分解释，不能否认用户实际升压结果，也不能认定芯片必须外部驱动DONE才能工作。
- 本轮仅核对资料与更新记录，不修改DONE方向、保护或固件，不构建烧录。来源：https://www.analog.com/media/en/technical-documentation/data-sheets/lt3751.pdf 。

## 74. 2026-10-20 对照DONE内部上拉配置

- 用户明确硬件没有DONE外部上拉。当前SRC gpio.c:95–98将PC13/KICK_ERR设为GPIO_MODE_INPUT/GPIO_PULLUP；SRC_Firmware.ioc亦保存PC13-ANTI_TAMP.GPIO_PuPd=GPIO_PULLUP及GPIO_Input。故当前软件已开启MCU内部弱上拉，并非只依赖不存在的外部电阻，也不是推挽高输出。
- 对照本地TIGERs主板网表，KICK-ERR/DONE接PG10；检索main2019及对应GPIO初始化/充电模块，未找到PG10/DONE配置上拉或读取路径。不能把对PG11/CHG的弱上下拉板检测误当作对DONE的处理。该参考依赖ADC电容反馈控制，并未用DONE作为充电门控；不声称TIGERs在软件中把DONE和CHG一同驱动高。
- 当前SRC与参考的区别是SRC内部上拉并读取DONE，低CAP充电时DONE低会进入DONE LOW保护。仅凭硬件无外部上拉不能判当前内部上拉失效；用户此前同时拉高的实际配置和测量仍待区分。本轮只说明并更新记录，不改固件、不构建烧录。

## 75. 2026-10-20 DONE内部上拉强度与RC判断

- 用户引用手册询问proper pull-up是否表示MCU内部上拉不足。核对ST STM32F405/407官方数据手册RPU：普通引脚弱上拉等效30/40/50kΩ（规定条件下）；ADI LT3751 DONE/FAULT设计章节对多数应用推荐100kΩ，并要求引脚电流不超过1mA。因此不能仅凭内部弱上拉断言静态强度不足：其等效阻值通常比100kΩ更低，在3.3V下低电平时电流约66–110uA。
- 若实板保留v13b图上C19=100nF，以线性电阻近似，内部上拉RC约3–5ms；节点从低电平释放恢复到最终值95%约9–15ms。这是量级估算而非实测，弱上拉并非精密线性电阻；可能影响10ms任务对DONE恢复的判定，需考虑实际电容、漏电和节点电压，不能将静态够用等同于完整时序已经验证。
- 当前用户实测NO RISE/CHG高3s，并非DONE LOW，不支持直接指定DONE上拉不足为本次根因。本轮不改上拉、RC、保护或固件，只更新认识；不建议用推挽高对抗DONE输出。
- 官方来源：https://www.st.com/resource/en/datasheet/stm32f407ig.pdf ，https://www.analog.com/media/en/technical-documentation/data-sheets/lt3751.pdf 。

## 76. 2026-10-20 DONE与CHG时序要求

- 再次核对LT3751官方手册第8及20页：CHARGE高于1.5V启动，低于0.3V停止，开启边沿上升时间10ns–10ms，不应直接随VCC缓升；DONE为开集电极状态输出，没有由MCU主动与CHG同步拉高的要求。
- 软件顺序建议：CHG先保持低、DONE先配置输入上拉，在电源有效且条件满足后CHG高，再判读DONE。DONE/FAULT的低锁存由CHARGE切换复位，不能要求锁存DONE必须先高才允许CHG升高，否则可能无法启动。CHG上升时DONE可正常低数微秒，须将启动瞬态及外部RC恢复与持续故障区分。
- 手册推荐用于抑制启动指示的RC时间常数5–20us；第75节按实板若保留100nF及30–50k内部上拉估算为3–5ms，明显更慢。软件是否需启动观察窗口应由实际DONE恢复/波形确定，不把该估算当实测或自动屏蔽所有启动故障。本轮只记录时序知识，不修改代码/硬件、不构建烧录。
- 来源：https://www.analog.com/media/en/technical-documentation/data-sheets/lt3751.pdf 。

## 77. 2026-10-20 按DONE/CHG时序修正初始化与启动判定

- 用户明确授权按第76节时序修正。源码基线仍d80b9f3及既有DIS极性/禁充显示等未提交工作树，保留用户15%吸球上限。
- 修改Core/Src/main.c、board_monitor.c、misc.c及Core/Inc/board_monitor.h、cfg.h。新增board_charge_gpio_init，在main SysInit阶段、MX_GPIO和其它外设前先PC4写低并设推挽输出，再PC13设输入上拉；board_monitor_init重复建立配置，DONE始终不作输出。
- KICKER_DONE_SETTLE_MS=20ms是结合内部上拉+图纸100nF恢复估算选取的软件初值，非芯片规定值。首次board_monitor_init后至少低20ms才允许CHG启动，等待原因显示STARTUP；每次真实CHG低转高请求记录新时间戳，前20ms暂缓DONE故障判断。包括30s重试与射球后恢复；不要求锁存DONE先变高才允许CHG边沿。DONE持续低到窗口结束仍报DONE LOW并30s后重试。
- 原ADC有效性、电池门限、温度、目标电压、quiet/shutdown、射球/80ms冷却及3s无进度保护保持有效；并非统一延迟所有保护。记录实际控制边沿请求，未宣称测得控制器CHARGE的上升时间或已建立12V供电有效检测。
- 新tests/test_done_startup.py在4S/6S共12项运行用例及初始化顺序检查通过，含初始低DONE可启动、瞬态恢复、持续低/重试、计时回绕及窗口内ADC/温度/电压保护。回绕测试显式注入INA转换就绪事件以避免初始battery_tick=0跨回绕的轮询等待混入DONE验证。test_charge_inhibit.py既有44项、DIS极性测试、OLED全13标签/轮换、音乐和四轮启动回归通过；既有DONE测试改为观察窗口后断言，非删除故障断言。旧完整吸球55%预期与15%上限不匹配未扩改。
- 最终Keil全工程构建0Error/0Warning，Code54934、RO-data5126、RW-data516、ZI-data29932，日志MDK-ARM/tmp/done_startup_build.log，正常SRC_Firmware.axf/hex更新。git diff --check通过，未烧录/实屏或实车测波形，不宣称NO RISE已解决。
- BOARD_MIGRATION.md补充本次时序与软件参数边界。启动GPIO接管前仍依赖硬件状态，20ms参数后续应以实板DONE恢复波形复核。

## 78. 2026-10-20 明确TIGERs实际时序与SRC新增等待的区别

- 用户再次要求详细对照TIGERs实际代码。重新直接读取固定目录E:/SRC/Tigers/Firmware-master/Firmware-master的shared/module/kicker.c（KickerInit/kickerTask/identifyKickerBoard/chargeControl/adcKickerUpdate）、main2019/main.c、tiger_bot.c、robot_impl.c及shared/drv/adc_kicker.c。
- 明确纠正表述：第77节新增首次保持CHG低20ms及每次CHG升高后DONE观察20ms，均为SRC自行增加的策略，TIGERs中没有这两项。不能说这是从TIGERs移植的时序，也不能把TIGERs识板时输入弱下拉10ms+弱上拉10ms合计20ms当作DONE稳定等待。
- TIGERs时序：main先初始化TIM13/14并建立DIS高关闭，再启动ADC任务；TigerBotInit绑定PG11为chargePin，KickerInit创建任务。kickerTask先在PG11上执行输入弱下拉10ms读IDR、输入弱上拉10ms读IDR，利用v13b的CHG外部10k下拉识板。成功后先写PG11输出锁存低，再设推挽输出/无拉。随后约1ms处理事件/检查充电条件；没有再等待20ms。
- TIGERs的DONE硬件对应PG10/KICK-ERR，但所查模块没有DONE输入配置、读取、启动屏蔽或DONE低禁充路径。名称EVENT_MASK_TIMER_PULSE_DONE及COOLDOWN_DONE指软件/定时器事件，不是LT3751 DONE输入，不能混淆。TIGERs不等待DONE先高，也不同时驱动DONE与CHG。
- TIGERs由RobotImplKickerAutoCharge(1)/测试/shell设置AUTO_CHARGE，主循环按ADC新鲜度、温度、快/慢滤波电容电压及KICKING/COOLDOWN等决策CHG；SRC为有效测量后自动充电，额外有电池有效/quiet门控和DONE故障门控。ADC采样TIGERs约1ms，SRC约10ms；参考默认200V、5V回差、80ms冷却，SRC180V/5V/80ms。3s/5V/30s为参考代码意图，BSRR状态读取缺陷仍沿用第71节边界。
- 当前SRC初始化helper先CHG低再DONE输入上拉，与TIGERs不使用DONE的路线不同；其20ms策略是为SRC额外读取DONE提供恢复裕量，不能作为解释TIGERs为何升压的依据。本轮只补对照与记录，未继续修改固件/参数，不构建烧录。

## 79. 2026-10-20 按用户要求撤回DONE策略并对齐CHG识板

- 用户明确质疑为何未直接采用TIGERs路线，实际按指定目录identifyKickerBoard迁移CHG输入弱下拉10ms读、弱上拉10ms读，通过后写低再切推挽输出/无拉。任务中首次执行一次；识板失败显示NO BOARD，禁止充电/普通射球及关机线圈脉冲。未识别时输入高不被误当作正在充电。
- 删除DONE输入上拉/读取/故障门控及两个额外20ms等待与相关配置宏；PC13保留未使用输入、无内部上下拉，.ioc同步。原DONE启动测试替换为test_charge_identify.py。SRC电池保护、180V目标/自动充电请求、3s进度保护、80ms冷却和DIS极性修复保留，不复制参考BSRR读取缺陷或上位机协议。
- 4S/6S识板12项（正常、无板、错误偏置、DONE切换不影响、进度重试、ADC故障）及充电相关44项、DIS极性、OLED、音乐、四轮启动回归通过；最终Keil日志MDK-ARM/tmp/tiger_charge_final_build.log，0Error/0Warning，Code55006、RO-data5118、RW-data508、ZI-data29932，正常AXF/HEX更新，未烧录/实测。旧55%吸球测试与用户15%不匹配未扩改。第77节的20ms等待已撤回，不再描述为现行实现。

## 80. 2026-10-20 应用户要求扩大DONE全目录追查

- 用户要求不能仅以充电模块搜不到断言没有DONE配置。扩大到固定目录的隐藏/忽略文件、主程序/bootloader、CMake源清单、公共GPIO初始化、直接MODER/PUPDR/OTYPER写入、端口基地址/别名、C/C++/汇编和启动钩子。目录统计2623个文件、1187个源码/配置文件；统计不代表逐行人工通读所有文件。审查脚本MDK-ARM/tmp/audit_done_source.py。
- 再查EAGLE Mainboard v13.sch：IC1为STM32H743Z，KICK-ERR网络接gate G/P10，对应封装125脚，即PG10，连接器PWR -38；不存在此次对照把DONE错认成PG11的情况。CHG是PG11，ADC CS是PG12。
- 对整个目录GPIOG引用穷举：main2019中PG0触摸、1显示复位、2/5/6/7/8图案识别、3/4软I2C、9/14USART6、11CHG、12ADC片选、13扩展器中断、15USB供电；没有PG10配置/读取/输出调用。公共init_hal.c只配置传入的端口和pin掩码，GPIODeInit和GPIOPinStInit无实际调用；无全端口通用循环配置PG10的证据。未发现PG10别名或固定GPIOG寄存器地址操作。
- main_bootloader.c仅配置PG15输入上拉；system.c处理时钟/MPU/SysTick，无GPIOG MODER/PUPDR设置；crt1.c的__early_init和__late_init均为空弱函数，全目录无其它覆盖。构建CMake定义main2019使用STM32H743xx；已有CMakeCache源路径确为用户指定目录，但build/main2019_Release/src/main2019仅见CMakeFiles，不能声称核验了现成ELF反汇编。
- 正向功能证据：shared/module/kicker.h:41/55的KickerData/Kicker只有chargePin，无donePin；tiger_bot.c:140只注入PG11；kicker.c:190的KickerIsCharged直接以capAvgFast >= maxVoltage-chgHysteresis判断充满，chargeControl:322也用ADC/温度/状态决策。TIMER_PULSE_DONE/COOLDOWN_DONE是定时器事件，不是LT3751 DONE。
- 扩大核查结果仍为：所给目录的main2019应用没有使用LT3751 DONE(PG10)的初始化与逻辑路径。这比“只搜kicker.c没看到”范围更完整，但不据此宣称其它未提供固件版本或用户实板测得的现象不可能。硬件预留状态线并不强制固件使用；该版充满反馈明确由ADC实现。
- 本轮扩大追查未再改固件；更新证据边界与记录，未构建TIGERs或烧录实物。

## 81. 2026-10-20 对照手册Start-Up与Start-Up Protection

- 用户提供LT3751 Start-Up原文。核对官方手册第10–12页：正常条件下CHARGE升高后约2us由芯片内部启动单稳态发起首次开关，之后维持开关允许直到目标或故障，不需要MCU驱动DONE或生成升压PWM。2us描述芯片内部响应，不是软件应延时2us再改另一引脚。
- 低输出电压（包括短路）可能不足以触发DCM比较器；内部启动保护以26kHz时钟和辅助电流比较器控制后续周期，并可增加周期间隔；失去DCM事件约500us可重新启动。因此启动阶段不必呈固定连续频率，不把稀疏脉冲直接当作MCU时序错误。
- 用户此前CHG高3s/NO RISE只能证明使能窗口内ADC未见足够增长，不能区分未开关、能量未积累或测压链未反映。后续如测量应以控制器CHARGE实际上升沿和升压MOS栅源脉冲为起点，不再为解决该现象增加DONE驱动或无依据的MCU启动等待。
- 本轮只核对手册并记录，不改固件/参数，不构建烧录。来源：https://www.analog.com/media/en/technical-documentation/data-sheets/lt3751.pdf 。

## 82. 2026-10-20 用户恢复DONE上拉后观察3.3V衰减至0V

- 用户反馈DONE配置默认上拉，上电后由3.3V衰减到0。重新读当前gpio.c:95–98及.ioc，二者现均为PC13输入/GPIO_PULLUP，替代第79节的无拉配置；尊重用户现有改动，不恢复NOPULL。本轮开头沿用旧无拉配置的假设已在读取后立即纠正。
- 当前board_monitor.c仍未读取或输出DONE；应用中未见初始化之后重新配置PC13/直接覆盖其上拉的明确路径。源码配置不替代运行时GPIOC.MODER/PUPDR或实际烧录镜像核验，未据此排除运行差异。
- 上拉不是强驱动高：LT3751的DONE开集输出可在充满、FAULT或启动短暂状态下拉低，内部弱上拉会被正常压低。因此电压下降不能单独证明上拉不足；若3.3V电源稳定且运行PUPDR确为上拉，单独100nF电容不能让已释放的节点长期保持0V，需要有下拉电流路径或上拉实际未生效。
- 若CAP仍近零且DONE持续低，需优先区分故障锁存/外部拉低与配置失效，不能称为正常充满。关键信息为下降时间尺度、与CHG上升沿的相对时刻、是否恢复以及FAULT实际状态。本轮不指定损坏元件、不改配置，不构建烧录。器件依据仍ADI LT3751第8/20页，https://www.analog.com/media/en/technical-documentation/data-sheets/lt3751.pdf 。

## 83. 2026-10-20 DONE在CHG之前约720ms指数衰减

- 用户补充并重复确认：DONE的3.3V至0衰减发生在CHG拉高之前，指数衰减持续约720ms。该事件不能用CHG上升沿触发、持续数微秒的LT3751启动低指示解释；不能据此直接认定充满或故障锁存。
- 当前再读源码仍PC13输入上拉，早期board_charge_gpio_init只处理CHG；MX_GPIO_Init在SRC_Robot_Init的50000000次忙等待前已配置DONE上拉。因此不能把该忙等待期间默认说成GPIO尚未初始化。运行时PUPDR、供电、镜像及实物连通性仍未核验。
- 720ms与图纸100nF+内部30–50kΩ的3–5ms量级不同。若实际节点高阻，100nF经1MΩ测量输入放电，时间常数约100ms，数百毫秒衰减到近零可能出现；只是条件性模型，不声称已证明探头导致，更不能将720ms全衰减时间直接等同于RC时间常数。
- 同时需排除示波器AC耦合：高通输入会将稳定DC上电阶跃显示成指数回零，屏幕回零不等于节点DC电压真归零。优先确认通道DC耦合、探头倍率，并用稳态直流读数/运行PUPDR区分真实放电与显示效应。若3.3V稳定、上拉实际有效且连通，单独100nF不能使DONE长期归零。
- 本轮不改代码、不构建烧录。资料：Keysight AC coupling说明 https://www.keysight.com/blogs/en/tech/bench/2018/10/18/when-to-use-ac-coupling-on-your-oscilloscope ，Tek探头输入阻抗 https://download.tek.com/datasheet/P2220-P2221-Passive-1x-10x-Voltage-Probe-Datasheet-51W179843.pdf ，LT3751手册同前。

## 84. 2026-10-20 用户确认供电正常且OVLO/UVLO未触发

- 用户确认FAULT无上拉，当前无法直接判读，并确认供电正常、OVLO/UVLO未触发。按该实测前提继续，不重复要求同一组供电/门限检查；尚无FAULT有效电平，不能把DONE低单独当作FAULT已证实。
- 修正上次答复过于倾向FAULT的推断：手册DONE还可由VOUT目标比较器置位；若供电门限已排除，误判充满/开关驱动与能量传递链需要区分。但此前DONE在CHG前720ms衰减本身并不能证明VOUT比较器已触发，不把两事件强行归为同一原因。
- 下一项区分性证据应是以CHARGE上升沿触发的升压MOS栅源VGS：从未开关、短暂开关后停止、持续开关但电容无增长对应不同支路。FAULT若后续要判读需有效上拉，不能用悬空电压作证。芯片手册明确FAULT还含内部UVLO，但本轮没有证据认定该情况发生，不据此否认用户供电实测。
- 本轮只记录和核对ADI LT3751 Pin Functions及Secondary Energy Transfer说明，不改固件/参数、不构建烧录。来源同前官方手册。

## 85. 2026-10-20 用户提供v13b相似案例及确认当前电池供电

- 用户截图：提问者采用TIGERs kickerboard线路/布局，在12V偏置及VTRANS条件下，CHARGE高后仅少数开关脉冲、电容仅升几V、DONE立即低。截图中Andre回复称设计按约24V电池供电调校，启动峰值约13A，建议将采样电阻增倍以减小初始电流，并提示门极电阻可能影响热失控。这是用户提供的案例/建议，不当作本机根因实测或已验证修复。
- 用户随后明确当前升压由100C-1550-6S1P电池供电。此前22V/1A台式源属于吸球测试，不得继续套用为本轮升压条件。电池标称倍率不能单独证明实际供能及所有连接，但在用户此前已确认供电/UVLO/OVLO正常的前提下，不优先重述“台式电源限流不足”。
- v13b图R18=8mΩ，LT3751充电模式名义峰值阈值106mV/RSENSE=>13.25A；16mΩ则6.625A。该值是初级/MOS开关电流峰值，不是电池平均电流、ADC监测电流或保险丝电流；启动保护还有辅助比较器，不能保证实际无过冲。增倍R仅是受控降低峰值的诊断/适配方案，用户未要求改硬件，本轮不操作。
- 该R18属于升压板，不是电机板吸球R27=39mΩ。截图所说gate resistor未指定具体位置；v13b原图升压Q1门极R14=0，而R21/R22=10Ω属于IGBT射球驱动，不应混淆后盲改。截图的24V供电指主功率母线条件，v13b图LT3751脚13仍接+12V，不据截图把偏置电源改成24V。
- 相似性边界：对方DONE在CHARGE高时变低且已观察少数门极脉冲，本机已明确DONE在CHG之前约720ms衰减，未取得对应升压Q1脉冲；两者不能仅凭DONE低合并为同一故障。后续优先核对CHARGE上升沿对应VGS及CSP-CSN波形，区分未启动、快速停机和持续开关无能量积累；不再改DONE驱动/无依据延时。
- 本轮核对ADI LT3751 Pin Functions/Current Sense Resistor/Start-Up Protection，来源https://www.analog.com/media/en/technical-documentation/data-sheets/lt3751.pdf ，只更新记录，不改固件/参数/硬件、不构建烧录。

## 86. 2026-10-20 用户实测上电IGBT门极约10V，修复TIM9后初始化覆盖

- 用户反馈一开机IGBT导通、G约10V。未确认该读数参考E还是地，不将其当成已测VGE；若G-E约10V，不能称为安全关闭。先停止高压充电并确认电容放电，不能在导通未排除时继续尝试升压。
- 本次基线仍d80b9f3501492870f11270f9beb8e1cf79a3d6bb及已有未提交工作树，保留用户PC13上拉、15%吸球及其它改动。重新读实际源码发现：TIM9极性仍PWM2/LOW，早期kicker_hold_outputs仍在，但HAL_TIM_MspPostInit的TIM9分支已恢复CubeMX形式GPIO_MODE_AF_PP/GPIO_NOPULL，覆盖GPIO高。CC1E/CC2E未开启时不能保证该AF输出高，存在使低有效DIS保持错误电平的明确初始化缺陷。该缺陷符合用户现象，但尚未用实板波形证明其为全部根因。
- 先运行原test_dis_polarity.py：8个运行场景通过，但MspPostInit静态保护断言失败，复现此前第72节已记录的再生成风险。此前保护依赖修改生成区不够稳健，不能继续声称当前源码初始状态已经正确。
- 实际修改Core/Src/tim.c：在TIM9_MspPostInit 0的USER CODE保留区调用kicker_hold_outputs，以#if 0屏蔽下方生成AF配置，并在TIM9_MspPostInit 1保留区以#endif闭合。初版return引起ARMCC不可达代码告警，因此最终使用预处理屏蔽。保持PA2/PA3先置输出锁存高再配置推挽输出/上拉；只允许kick_pulse在准备好PWM输出后连接选中通道AF。未改射球力度、时长、充电条件或DONE。
- 再次直接读取本地TIGERs src/shared/hal/h7xx/hal/timer_opm.c：TimerOpmInit先CCMR1=OCM_DISABLED(101强制有效)、CCER=CC1E、EGR=UG，再GPIO AF/上拉/推挽，因此接入AF前已建立高输出。SRC原问题路径并不具备该前提；本轮用GPIO高保持实现关闭，不声称寄存器实现与TIGERs完全相同。
- tests/test_dis_polarity.py增加提取并执行实际TIM9 MspPostInit分支的msp_startup用例，检查保留区guard以及#if 0/#endif包围AF。在4S/6S共10个DIS运行场景通过，充电联锁44项及CHG识板12项通过。宿主模型不能替代实际PA2/PA3及IGBT G-E波形。
- 边界：MCU复位到软件接管前的引脚状态依赖硬件，软件本轮保护不能保证该窗口；本轮未烧录、未取得新实板测量，也不能据此宣称NO RISE已解决。Keil最终构建结果另补。
- 最终构建另发现当前uvprojx在Application/User/Core和Drivers/CMSIS两组重复包含同一Core/Src/system_stm32f4xx.c，首次链接报8个重复符号错误。仅删除前者无独立选项的重复File条目，保留后者及用户其它工程设置；未改SystemInit实现。最终Keil日志MDK-ARM/tmp/dis_msp_guard_final_build.log：0 Error/0 Warning，Code54982、RO-data5118、RW-data504、ZI-data29936，2026-10-20 19:20生成SRC_Firmware/SRC_Firmware.hex。未烧录。
- 本轮tim.c及DIS测试git diff --check通过；uvprojx原有文件尾额外空行仍有格式提示，未为此重写用户工程其它内容。最终预处理保护版DIS共10场景重新通过。

## 87. 2026-10-20 复核初始化、射门和关机放电的极性一致性

- 用户追问初始状态改变是否同步射门、放电等操作。重新核对action.c、misc.c、board_monitor.c、tim.c和.ioc全部SHOOT/CHIP/TIM9引用：set_shooter经shoot_on/chip_on到start_kick，再到kick_pulse；关机放电discharge_on也调用同一kick_pulse；shooter_off经shoot_off/chip_off到kicker_stop，保持两路GPIO高。未找到绕过该底层直接输出DIS的其它应用路径。
- 发现本轮读取时tim.c时间19:28:51、.ioc时间19:28:47，TIM9极性均已为HIGH，与上一轮19:20构建时LOW不一致；不推断具体修改来源。MspPostInit USER CODE高保持保护仍保留。运行test_dis_polarity：10个模拟运行场景可通过，但.ioc LOW静态断言失败，说明不能仅凭此前验证声称当前配置一致。
- 按现有低有效修复目标，将MX_TIM9_Init的共享通道配置恢复TIM_OCPOLARITY_LOW，并同步.ioc中TIM9.OCPolarity_1/2为LOW。TIM_OCMODE_PWM2保留，两通道使用同一配置。board_kicker_ready和discharge_on原有CC1P/CC2P均置位检查保持有效，错误极性时拒绝启动。
- 实际时序：闲置/停止两路高；普通射门或挑射只给选中一路低脉冲，另一路保持高；关机线圈放电同样选中一路低脉冲；单脉冲计数回零后恢复高。CHG在开始脉冲前低，发射/80ms冷却期间禁止充电。原协议力度、脉宽映射及关机放电算法不变。
- 增强test_dis_polarity静态检查实际MX_TIM9_Init必须PWM2/LOW，不只查.ioc和模拟CCER。最终10个DIS运行场景及源码/.ioc一致性检查通过；test_charge_inhibit既有44项通过。源码和.ioc的git diff --check通过。等待本轮Keil构建结束后补结果；未烧录或取得新实板测量。
- 本轮完整Keil构建完成：MDK-ARM/tmp/dis_operations_build.log，0 Error/0 Warning，Code54982、RO-data5118、RW-data504、ZI-data29936，正常SRC_Firmware.axf/hex已更新。此结果对应恢复LOW后的当前源码，未烧录。

## 88. 2026-10-20 核对踢球前关闭升压及并发联锁

- 用户询问踢球是否保证先禁用升压。直接核对当前kick_pulse：kicker_stop保持两路DIS高之后，先HAL_GPIO_WritePin(CHG_EN, RESET)，再装载CCR/更新定时器、使能通道并接入AF，最后启动计数；平射、挑射及关机放电共用此函数。可以确认软件命令顺序为先CHG低再发射低脉冲。
- start_kick和discharge_on调用kick_pulse均处于FreeRTOS临界区；board_monitor最终写CHG也在临界区内，且在写前重新执行charge_pulse_reasons检查TIM9运行状态、pending射球记录和80ms冷却，避免旧采样决策在射球后重开升压。pending使短于监控周期的脉冲也不会漏掉冷却。
- 保证范围是MCU控制输出顺序和软件联锁。当前没有独立确认LT3751功率级已停开关的反馈，也没有固定且经实板验证的关断等待参数；不能将先写CHG低等同于已实测最后一周期能量传递结束。本轮仅源码核查与记录，不改固件、不重新构建或烧录。

## 89. 2026-10-20 修复SystemInit及系统时钟符号缺失链接错误

- 用户报告L6218E缺少SystemInit、AHBPrescTable、APBPrescTable、SystemCoreClock。直接核对当前Core/Src/system_stm32f4xx.c：文件存在且定义上述符号；当前uvprojx无任何system_stm32f4xx.c条目，Drivers/CMSIS组为空，19:37:50生成的链接输入列表也没有系统源码对象。与第86/87节构建时保留一个条目的状态不同，不推断当前缺失的修改来源。
- 仅修改MDK-ARM/SRC_Firmware.uvprojx，在Drivers/CMSIS组恢复../Core/Src/system_stm32f4xx.c的C源文件条目。保持工程字节换行风格，不重写其它配置、不改系统源码或射球/充电行为。XML解析校验系统文件恰好一个条目、路径存在，避免重复符号和缺少符号两个相反问题。
- 启动实际Keil构建，日志MDK-ARM/tmp/restore_system_build.log，最终结果补于下文。未烧录。
- 最终完整Keil构建已通过：0 Error/0 Warning，Code54982、RO-data5118、RW-data504、ZI-data29936，编译日志明确包含compiling system_stm32f4xx.c，2026-10-20 19:40:58更新SRC_Firmware.axf/hex。实际验证不再有用户报告的四个未定义符号；未烧录。

## 90. 2026-10-20 用户确认主控侧DIS仍LOW，缩小软件接管空窗并要求运行时证据

- 用户确认在CubeMX将SHOOT/CHIP设为上拉，但DIS仍默认LOW；后续确认测点为主控PA2/PA3或连接器。当前.ioc两脚PULLUP且TIM9两通道LOW，TIM9 MspPostInit保留区屏蔽AF保护仍在；生成AF块里的PULLUP因#if 0不执行。实际闲置配置由kicker_hold_outputs主动写PA2/3高、OUTPUT_PP/PULLUP，不是仅靠弱上拉。
- 当前main早期SysInit中已无kicker_hold_outputs，调用移至USER CODE 2（所有外设初始化之后）；与第87/89节验证时序不同，不推断修改来源。将高保持调用补入USER CODE Init，即HAL_Init之后、SystemClock_Config及所有MX外设之前，保留已有后续调用。该变更只缩小软件启动接管前空窗，不能据此解释正常运行后的持续LOW。
- 对Core源码GPIOA、GPIO_PIN_2/3、SHOOT/CHIP、MODER/ODR/BSRR及反初始化路径重新检索：主控宏仍SHOOT=PA2、CHIP=PA3，LED和编码器对应其它脚/端口；未发现新的闲置主动拉低两脚路径。uvprojx的main/misc/tim路径指向当前Core/Src。未以此排除实际烧录/运行配置或外部电气问题。
- 加强DIS测试：msp_startup先将模拟两脚锁存与模式清零，再执行实际MspPostInit分支；检查早期高保持必须位于USER CODE Init且在SystemClock_Config前。4S/6S共10个运行场景及相关静态检查通过；准备实际Keil构建及机器码核查。
- 已向用户请求在OLED正常运行且无射球时暂停读取GPIOA MODER/ODR/IDR/PUPDR。预期闲置MODER&0xF0=0x50，PUPDR&0xF0=0x50，ODR&0x0C=0x0C，IDR&0x0C=0x0C；必须用运行时值区分配置/锁存与引脚采样，不再仅以源码和宿主测试宣称实板已修复。未烧录，无新运行时寄存器或波形证据。
- 最终Keil构建：MDK-ARM/tmp/dis_early_takeover_build.log，0 Error/0 Warning，Code54986、RO-data5114、RW-data504、ZI-data29936，19:57:49更新HEX。使用实际ARMCC fromelf反汇编当前AXF（tmp/dis_early_takeover_disasm.txt），main明确依次调用HAL_Init、kicker_hold_outputs、SystemClock_Config；kicker_hold_outputs中RCC GPIOA时钟使能、GPIOA基址0x40020000、引脚掩码0x0C/SET=1传给HAL_GPIO_WritePin，再以模式1/上拉1配置两脚，证明实际镜像含所述逻辑。main及DIS测试git diff --check通过。仍待用户运行时寄存器，未确认持续LOW根因，也未宣称此次前移已解决实板问题。

## 91. 2026-10-20 用户提供PA2/PA3运行时GPIO证据

- 用户Keil Watch截图：GPIOA->MODER & 0xF0=0x50，PUPDR & 0xF0=0x50，ODR & 0x0C=0x0C，IDR & 0x0C=0x0C。该暂停时刻两脚均普通输出、上拉，输出锁存与引脚数字输入采样均高；实际不是此前怀疑的GPIO复用覆盖或锁存低状态。OTYPER未在截图中直接测量，当前源码设OUTPUT_PP，不将截图单独称作OTYPER实测。
- 该证据仅覆盖截图时刻，不能排除上电短暂低、其它运行时刻变化，也不能将IDR数字高直接当作精确3.3V模拟测量。与此前用户主控PA2/PA3或连接器测LOW需用同一时刻的MCU焊盘/连接器对主控GND电压及物理连通性区分；不能继续仅为解释该截图而修改PWM极性。
- 本轮不改固件，不构建或烧录。后续优先对应测点/时刻；若主控侧确为高而IGBT仍导通，继续沿连接器、隔离器输入/输出及驱动级定位，不据此宣称高压板已安全关闭。

## 92. 2026-10-21 KICK_POLARITY可配置极性审查（待用户保存实际修改）

- 用户说明实际栅极驱动芯片不同，新增KICK_POLARITY表示DIS1/2开启踢球的电平，已修改TIM9和初始化，要求检查遗漏，尤其关机放电。以此为新硬件前提：历史第72至91节的固定低有效结论只适用于当时v13b驱动链，不再可直接套用于所有实物。
- 当前磁盘仍d80b9f3加既有未提交树。读取Core/Inc/cfg.h、Core/Src/tim.c、misc.c未找到KICK_POLARITY；全工程搜索也未找到宏定义或使用。cfg/misc文件时间仍2026-10-20 15:08:42，tim为2026-10-20 19:53:06。已请求用户保存到本工程或提供另一份路径，不能把旧文件审查称为已检查用户新实现。
- 旧文件明确的兼容遗漏：board_monitor.c:658 board_kicker_ready与misc.c:450 discharge_on均要求CC1P|CC2P全置位（固定PWM2/LOW），若改PWM2/HIGH会拒绝普通射球和关机放电，后者由update_discharge转BOARD_DISCHARGE_OUTPUT并shooter_off停止，不能误称已放空电容。
- 其它必须在新版本复核的点：kicker_hold_outputs的锁存和上下拉应对应非激活电平，且先写锁存再切输出；kick_pulse:416连接AF仍写死PULLUP，应与空闲策略一致。shoot_off/chip_off、HAL启动失败、discharge_fail、discharge_kill、shutdown初始stop_loads均经kicker_stop/hold_outputs，helper正确适配才能覆盖全部退出路径。
- 保留PWM2及CCR=period-ticks时，输出极性应对应实际激活电平，CNT=0与OPM结束应为非激活电平；若用户同时改PWM1，必须重新核对CCR和脉宽，不能只翻转CCER判断。CHG和KILL为独立信号，不随DIS极性宏翻转；先CHG低再启脉冲、射球和冷却禁充策略不因DIS高低改变。
- 既有tests/test_dis_polarity和test_board_migration模拟器写死高空闲/低有效、P位全置位及上拉，不能作为双极性实现已通过的证据。应待实际修改到位后覆盖两种极性、双通道、普通射球/关机放电、停止/失败、OPM结束与错误配置拒绝；本轮仅审查和记录，不改固件行为、不构建或烧录。

## 93. 2026-10-21 重新读取确认cfg.h第31行KICK_POLARITY=1，审查当前实际引用

- 用户指出已保存于cfg.h第31行。本次重读确认Core/Inc/cfg.h:31为#define KICK_POLARITY 1，文件时间2026-10-21 14:01:00；第92节“未找到”仅描述前次读取时刻，不再描述为当前状态。
- 再次全Core检索KICK_POLARITY只命中此定义，没有任何使用；tim.c仍2026-10-20 19:53:06，misc.c仍2026-10-20 15:08:42。按当前磁盘内容审查，不推断用户编辑器状态，也不声称已看到另一个版本。
- 按用户定义1=DIS高电平开启：misc.c:355的kicker_hold_outputs固定SET与此冲突，导致初始化、停止、HAL启动失败、关机放电失败/冷却/结束经共同helper发出的实际为激活电平；这是当前审查最严重项。misc.c:358与416固定PULLUP也不随空闲低适配。
- tim.c:497/499仍PWM2/LOW，配合CCR=period-ticks输出低脉冲并在OPM回零高，与KICK_POLARITY=1目标不符。若保留PWM2及脉宽算法，应由宏选HIGH或LOW，结束电平取反。
- misc.c:450 discharge_on及board_monitor.c:658 board_kicker_ready仍固定要求两路P位全置位；改TIM9为HIGH后会拒绝射门及关机放电，后者进入BOARD_DISCHARGE_OUTPUT。当前未改TIM9时检查可通过，但仍只会按旧低有效波形执行，不能表述为宏=1本身已使检查失败。
- 审查结论：定义已保存，但当前应用源码尚未消费该宏，无法称双极性支持完成。先禁CHG、发射/冷却禁充以及关机电压反馈算法本身无需反相；仅按用户本轮检查要求报告并记录，不擅自修改固件、不构建或烧录。

## 94. 2026-10-21 重读确认用户已适配初始化和TIM9，剩余关机放电阻断

- 用户要求再看。当前misc.c时间2026-10-21 14:04:02，tim.c时间14:04:10，已实际读取新增分支。第93节的“只有定义未使用”不再适用于当前版本。
- 已正确适配：kicker_hold_outputs在KICK_POLARITY=0时SET/PULLUP、其它值时RESET/PULLDOWN，先写锁存后配推挽；TIM9_Init 2在宏=1时将PWM2两通道重新配置为HIGH。宏=0沿用生成区PWM2/LOW。当前0/1两种值的空闲与有效脉冲逻辑方向正确，CCR=period-ticks及OPM回零算法无需因此翻转；相关高空闲/低有效注释已过时。
- 仍存在的阻断：board_monitor.c:658 board_kicker_ready与misc.c:458 discharge_on均固定要求CC1P|CC2P全部置位。宏=1时实际HIGH将两位清零，普通射门许可为假，关机放电返回0，update_discharge:442进入BOARD_DISCHARGE_OUTPUT。需按宏比较预期P位：0时mask，1时0，不可删除极性检查。尚未按用户仅检查的要求改代码。
- 已覆盖退出路径：shoot_off/chip_off、HAL启动失败、关机stop_loads、放电完成/冷却/失败/强制关机均经kicker_stop/hold_outputs，因此本次helper修改使宏=1时这些路径返回GPIO低关闭，不再沿用第93节“停止高激活”的当前结论。
- 配置不一致：kick_pulse接入AF的config.Pull仍固定GPIO_PULLUP，应随空闲电平选择。已使能推挽外设的有效低输出不会仅因内部弱上拉就变高，不能将此列为当前拒绝放电的直接原因。
- HAL实现核对：HAL_TIM_PWM_ConfigChannel在每次配置通道时重新开启OC1PE/OC2PE；用户宏=1分支二次调用使此前DISABLE_OCxPRELOAD失效。当前kick_pulse写CCR后主动EGR UPDATE，仍能装载正确比较值，未发现由此造成脉宽错误的证据；建议将禁预装载移到最终配置之后保持两极性一致。
- 既有DIS测试模型和静态断言仍只适用低有效（甚至断言tim.c不得出现HIGH），尚不能验证此次双极性新实现。当前分析仅源码及HAL核对，不修改固件、不运行旧测试冒充双极性验证、不构建/烧录。

## 95. 2026-10-21 按用户授权修复KICK_POLARITY四项遗漏并验证双极性

- 用户明确要求修改第94节四点。基线d80b9f3501492870f11270f9beb8e1cf79a3d6bb加已有用户工作树；保留cfg.h的KICK_POLARITY=1及用户初始化/TIM9分支、180V目标、15%吸球和其它既有改动，不改变参数含义或引脚。用户实际驱动差异已取代历史固定低有效假设。
- Core/Src/board_monitor.c board_kicker_ready与Core/Src/misc.c discharge_on：保留双通道极性保护，但按宏比较预期P位（0=>CC1P|CC2P，1=>0）。高有效下现在允许匹配配置的普通射门及关机放电，错误配置仍拒绝，不通过删除保护绕过。
- misc.c kick_pulse接入AF时Pull按宏选择：高有效用PULLDOWN，低有效用PULLUP，与用户hold_outputs空闲电平一致；调整相关只适用高空闲的注释为inactive。保留原PWM2/CCR=period-ticks、先CHG低再启动计数和冷却禁充。
- tim.c将两条DISABLE_OCxPRELOAD从前面的生成区移至TIM9_Init 2 USER CODE中最终通道重配置之后（#endif后），两种极性最终都禁预装载。保留用户宏=1时二次配置HIGH的做法，以及MspPostInit跳过生成AF的保护。
- tests/test_board_migration.py的HAL模型改为按当前宏计算空闲电平、上/下拉和预期P位；每次HAL启动射球前检查两路已在非激活GPIO状态且CHG低。保留原电池/ADC/放电故障模型，不扩大修改此前固定55%吸球预期与现行15%不匹配问题。
- tests/test_dis_polarity.py扩展为0/1极性x4S/6Sx12场景=48项。提取执行真实TIM9_Init 2、MspPostInit分支和生产射球函数；模拟HAL每次ConfigChannel重开预装载副作用，断言最终两路预装载关闭及P位正确。覆盖两通道最短/最长脉冲、OPM回零关闭、HAL失败、IRQ保持、错误极性拒绝、双通道关机放电、正常放空、线圈回退、无进度/传感器/输出/超时失败及其关闭电平。测试临时cfg副本遍历极性，finally恢复测试副本；不修改生产cfg宏值。
- 双极性48项及test_charge_inhibit的44项共92个运行场景通过；后者还包含原普通射门/关机放电、故障、冷却和短脉冲联锁回归。git diff --check对本轮生产文件与测试通过。宿主测试不是实板验证。
- 最终实际Keil构建日志MDK-ARM/tmp/kick_polarity_final_build.log：0 Error/0 Warning，Code55022、RO-data5118、RW-data504、ZI-data29936；2026-10-21 14:12:02生成正常SRC_Firmware.axf/hex。Keil构建对应当前宏1，宏0由宿主C编译/运行验证；未烧录、未测新硬件实际DIS/IGBT波形，不宣称实板安全或升压问题已解决。
