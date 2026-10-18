# SRC RoboCup SSL 下位机：项目背景、TIGERs 对比与持续工作记录

更新日期：2026-10-18（Asia/Shanghai）。本文件是项目长期知识入口；自动化协作入口见同目录 AGENTS.md。

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
