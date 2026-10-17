# SRC RoboCup SSL 下位机：项目背景、TIGERs 对比与持续工作记录

更新日期：2026-10-16（Asia/Shanghai）。本文件是项目长期知识入口；自动化协作入口见同目录 AGENTS.md。

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

### P0：当前车型宏存在确定的预处理问题

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
