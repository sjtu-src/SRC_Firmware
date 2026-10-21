> 最新状态（2026-10-20）：用户要求对齐TIGERs后，SRC已撤回DONE门控及两个额外20ms等待，改为CHG输入弱下拉10ms/弱上拉10ms识板，通过后先写低再切推挽输出；识板失败显示NO BOARD。下面“第77节之后”的表格为已被替代的中间状态，当前以AGENT.md最新节和BOARD_MIGRATION.md末节为准。DIS低有效修复保留。
> 后续状态：2026-10-20 用户已授权并完成DIS修复。当前TIM9为PWM2/LOW，启动/停止/故障为GPIO高，成功装载后延后接AF，安全守卫/关机放电/CubeMX参数及测试同步更新；最终构建0错误0警告，尚未实板验证。下文保留修复前审计证据，不再代表当前极性，详见AGENT.md第72节及BOARD_MIGRATION.md最新节。
# Kicking Device v13b 升压、引脚与时序审计

日期：2026-10-20。SRC基线d80b9f3501492870f11270f9beb8e1cf79a3d6bb，加第69节禁充显示/冷却未提交改动。用户确认当前射球板是TIGERs Kicking Device v13b，CHG确实拉高约3s，OLED报NO RISE，此前BAT22.5V/CAP0.01V。本轮只分析、修订记录，不改变控制行为。

参考：本地Firmware-master/src/main2019 + src/shared；electronics-master/v2020/PDF/Kicking Device v13b.pdf两页及同名EAGLE .sch网表。图纸日期2020-04-14，无可靠固件Git提交号，不代表TIGERs现役版本。SRC主板参考Version3.0/Mainboard_2026-10-10.pdf。powerboard_schematic.pdf是电机/电源板，不是该独立射球板；POWBRD_Flyback/SRC_PowerBrd_Flyback.pdf是旧16V/VFB方案，不作为当前实物接法。

## 1. 结论与优先级

1. CHG两边均高有效、推挽，正常充电时保持高，不是PWM。不需要给LT3751发送SPI配置，也没有另一根由MCU控制的升压使能/复位线。
2. **发现确定的适配冲突：v13b的DIS1/DIS2是低有效，SRC当前按高有效产生脉冲并在OPM结束后保持低。** 本地TIGERs从定时器寄存器到FAN3229反相链一致；SRC此前“PWM2/HIGH返回低即关闭”的假设不适用于该板，必须修订。对已确认的v13b，需要将启动/运行/停止/故障/关机放电的DIS关闭状态统一为高、射球为受控低脉冲；不能只改一个CCER极性而保留其余时序假设。
3. 若DIS低真的到达板上隔离器及驱动器，FAN3229可能将IGBT门极拉高，放电支路可阻止电容积累电压。这是与NO RISE相关的优先候选，但尚无DIS/门极实测，不能宣称根因已证实。初始CCER关闭时的真实PA2/PA3电平也不能仅从PWM2公式推断。
4. SPI2实际SCK为TIGERs4.5MHz、SRC5.25MHz，均在ADC122S051手册3.2–8MHz范围。没有依据仅因分频不同就认定SRC不升压；应先解决DIS极性和关闭状态。
5. TIGERs此份源码GPIOPinIsSet读BSRR，存在参考实现缺陷；不能把其3s/5V看门狗按设计文字当成已证明正确运行，也不应照搬该宏。

## 2. 控制架构：软件允许，LT3751自行开关

主控CHG -> LT3751.CHARGE(8) -> LT3751感知电源/变压器/电流 -> HVGATE -> Q1/变压器/整流 -> 高压电容。

反馈：电容分压R19=10M/R20=133k -> CAP-RAW -> MCP6002缓冲 -> ADC122S051 IN2 -> SPI -> 主控软件停充。

旁路：DIS1/DIS2 -> SI8620（非反相隔离）-> FAN3229的负输入 -> IGBT -> 放电线圈支路。这条链在“没有射球命令”时也必须处于可靠关闭状态。

外部LT3751手册确认CHARGE用于启停/启动新周期；DONE/FAULT为低有效锁存指示，切换CHARGE清除，启动时也可短暂拉低数微秒。来源：https://www.analog.com/media/en/technical-documentation/data-sheets/lt3751.pdf 。没有发现TIGERs固件控制HVGATE/LVGATE的路径，不能用电机PWM代码解释升压开关频率。

## 3. 完整引脚对应（射球板X1接口）

| X1脚/信号 | TIGERs MCU及配置 | SRC MCU及配置 | 时序/意义 |
| --- | --- | --- | --- |
| 1 KICK-NSS/CS | PG12，GPIO推挽，速度档1（宏25MHz），无内部上下拉；先写高后切输出 | PB12，GPIO推挽，VeryHigh档，内部上拉；先写高后配置 | 空闲高；一次64字节ADC批次期间低；不是LT3751使能 |
| 2 KICK-SCK | PB13 AF5，推挽，上拉，速度档1 | PB13 AF5，推挽，无拉，VeryHigh档 | SPI2 mode3，空闲高，第二沿采样；TIGERs每次传输前配置SPI，SRC在MX_SPI2_Init配置 |
| 3 KICK-MISO | PB14 AF5，上拉，速度档1；总线配置otype推挽，但主模式下该信号为接收 | PB14 AF5，无拉，VeryHigh档；HAL统一AF_PP，主模式接收 | ADC到MCU；不可把统一AF_PP字样误解为主控主动驱动MISO |
| 4 KICK-MOSI | PB15 AF5，推挽，上拉，速度档1 | PB15 AF5，推挽，无拉，VeryHigh档 | 传ADC下一帧通道选择，非LT3751命令 |
| 5 KICK-CHG | PG11，检测阶段输入弱下拉10ms/弱上拉10ms；通过后先写低，再推挽输出、无拉、速度档1 | PC4，先写低，再推挽输出、无拉、High档；无板识别 | v13b板上R2=10k下拉，CHG直接接LT3751脚8；高允许，低停止 |
| 6 KICK-DIS1（平射） | PF8 AF9/TIM13，推挽、上拉、速度档1；初始化强制有效OCM=101，CC1E=1、CC1P=0，即输出高 | PA2 AF3/TIM9_CH1，推挽、无拉、Low档；PWM2/High，初始停止通道 | TIGERs高关闭、低脉冲；SRC有效通道在CNT0及回零后低，极性相反 |
| 7 KICK-DIS2（挑射） | PF9 AF9/TIM14，其余同DIS1 | PA3 AF3/TIM9_CH2，其余同DIS1 | 同一反相冲突；两路都需核验，不能只改平射 |
| 8 KICK-DONE | TIGERs主板接PG10/KICK-ERR；本地kicker.c没有读取/初始化这一输入的路径 | PC13/KICK_ERR，输入、内部上拉，10ms轮询，不用EXTI | 直接连LT3751 DONE(7)，板上C19=100nF对地；SRC以DONE低+低CAP触发30s重试 |
| 9 GND | 共地 | 共地 | CHG/SPI/DONE参考；非高压CAP正端 |
| 10 MAIN-3V3 | 为SI8620输入侧VDD1供电 | 主控侧供电接口，实际线束须对应 | 无固件可独立开关；不是ADC主电源 |
| 11 +12V | LT3751 VCC、FAN3229和板上3.3V LDO输入 | 当前使用同一v13b，要求线束提供对应电源 | 硬件供电；不由CHG产生12V |
| 12 MAIN-VCC | 经板上F1=8A slow到VCC/变压器输入 | 需对应SRC电池母线 | INA226显示BAT不证明该端和+12V都已到达板上 |

GPIO速度档是输出边沿/驱动配置，不等于软件切换频率。TIGERs宏25MHz不能按H7实际引脚最大频率作精确波形指标。

## 4. 其它电源相关引脚（不在射球板12针上）

| 功能 | TIGERs | SRC | 对升压的影响 |
| --- | --- | --- | --- |
| INA226 SCL/SDA | I2C2，PF1/PF0，AF4开漏、无拉、速度档0；底层TIMINGR注释200kHz | I2C3，PA8/PC9，AF4开漏、无拉、VeryHigh档，100kHz | 都读0x40，0x42DF配置。SRC另查ID、转换完成标志和100ms新鲜度，作为充电前置条件 |
| SMB_ALERT | 本地PwrINA226驱动周期读寄存器，无对应ALERT输入依赖 | PB1，输入上拉、下降沿EXTI，ISR只记标志，任务仍轮询 | 无需ALERT边沿才能首次升压 |
| PDN | PD3，输入上拉、下降沿EXTI，按键低约50ms确认 | PD3，输入上拉、下降沿EXTI；任务低50ms确认 | 关机请求停止充电并进入受控放电 |
| KILL | PD6，先写高释放，开漏输出、上拉、速度档0；关机拉低100ms后释放 | PD6，先写高，开漏、上拉、High档；放电完成/强制关机才低100ms | 高=释放，不是升压启动线；拉低会切整机电源 |
| USB-powered | PG15输入上拉，整机电源状态用 | SRC无同一输入管理链 | 属于整机状态差异，不是v13b缺少GPIO |

## 5. 上电、第一次充电与重试的实际时序

### TIGERs

1. main使能GPIO/DMA等时钟，SystemInit配置PLL；chSysInit后初始化I2C2、SPI2、TIM13/14。
2. TIM13/14初始化：OPM、PSC99、ARR65535、CCR1=1、OCM=101强制高、CC1E=1；EGR更新清标志后才接AF。两路在充电前就已建立高电平关闭态。
3. SPI2脚设AF5，上拉。DevAdcKickerInit设PG12片选先高；adcTask约每1ms读一次ADC。
4. TigerBotInit创建PowerControl、Kicker任务。Kicker任务identifyKickerBoard：CHG输入下拉10ms -> 读IDR；输入上拉10ms -> 读IDR。v13b的10k下拉应让两次均低。不是“先低10ms再强驱动高10ms”，也不是LT3751的特殊启动脉冲。
5. 识别通过，CHG输出锁存先低，后切推挽输出。初始charger.mode为IDLE（全局对象零初始化）；上层RobotImplKickerAutoCharge(1)/测试或shell chg 1才改AUTO_CHARGE。
6. ADC有新样本更新快/慢EMA，kicker约1ms检查允许条件；温度、ADC时间戳、目标回差、kick/cooldown及模式允许后CHG置高，并持续保持。
7. 到最大电压（默认200V，配置可覆盖）或任何禁止条件，CHG置低；低于最大值-5V才重新请求。
8. 设计代码包含3s未增加5V停30s重试，但其GPIO读取宏存在下述问题，不能确认本地版本在硬件上真的如此执行。

### SRC当前

1. HAL/系统时钟后，MX_GPIO_Init先写CHG低、CS高、KILL高，再配置对应输出；DONE输入上拉。
2. MX_TIM9_Init设PSC3359、ARR635、PWM2/High、CCR1/2初值1，停止两通道后接PA2/PA3 AF3；没有像TIGERs那样先建立强制高的射球关闭状态。
3. MX_SPI2_Init设mode3/8bit/MSB，APB1/8。SRC_Robot_Init -> board_monitor_init再次写CHG低/CS高，准备32个ADC帧。
4. RTOS BoardUpdate每10ms采ADC/INA226并判断；不等音乐、自检结束，也不要求无线AUTO_CHARGE字段。
5. 新鲜电池>19.8V、有效ADC、温度未热锁存、无quiet/shutdown/射球/冷却/故障、CAP<175V后CHG高。
6. CHG高期间，每增加5V重置进度计时；3s无该进度记录NO RISE并拉低，30s后再次试充。DONE有效+低CAP时优先记录DONE LOW。
7. 用户已确认高3s，与该看门狗一致。当前“软件完全没给CHG使能”的旧表述不再成立，但尚未确认测点在MCU还是LT3751脚8，也未实测DIS门极状态。

## 6. SPI逐帧时序与换算

- 两边CS均覆盖整个64字节批次，而不是每个8bit字节拉高一次；总计512个SCK、32个16bit转换帧。
- 前16帧发送00 00，选择通道0；后16帧发送08 00，选择通道1。ADC选择有流水线/切换建立过程，因此每组丢弃前2帧。
- 接收格式是前部填充位+12bit数据；两边仅接受小于4096的16bit值，每通道至少4个合法样本。TIGERs以uint16缓冲后REV16，SRC以高字节<<8|低字节，语义相同。
- IN1（通道0）连接MCP9701A温度；IN2（通道1）连接缓冲后的CAP。T=(6600*normalized-800)/39；Vcap=251.42*normalized。未发现通道互换、移位或端序迁移错误。
- TIGERs系统PLL2P=72MHz，DIV16=>4.5MHz，64字节纯移位约113.8us；SRC APB1=42MHz，DIV8=>5.25MHz，约97.5us。HAL轮询字节间空隙、DMA和调度开销不包含在此计算，必须用波形判断实际连续性。
- TIGERs DMA收完IRQ拉高CS、关SPI，唤醒ADC任务；SRC阻塞HAL结束后拉高CS，失败也释放CS。TIGERs传输超时100ms，SRC3ms；TIGERs有效样本10ms未更新即禁充，SRC一次读失败立即禁充。
- TIGERs快/慢EMA为0.8/0.98，SRC只有批次内均值，无跨批次EMA。该差异可能改变噪声/停充响应，但不能解释已确认CHG高3s时为什么物理升压一定不工作；ADC数值有效也不代表电容实际电压准确。
- TI器件手册的SCK范围3.2–8MHz覆盖两者，不能盲改SRC分频到16（当前APB1/16只有2.625MHz）。来源：https://www.ti.com/lit/ds/symlink/adc122s051.pdf 。

## 7. 射球与升压互锁：确定的极性冲突

TIGERs代码位置：shared/hal/h7xx/hal/timer_opm.c:3、:6、:41；main2019/sys/tim13.c、tim14.c；shared/module/kicker.c:startKick。

- OCM_DISABLED=101是“强制有效”，CC1P=0所以有效=高，并非“强制低/关闭通道”。
- OCM_PWM=110是PWM1。PSC99配100MHz定时器输入得到1us/tick；CCR1=1、ARR=duration、CEN启动后，先约1us高，随后低，溢出CNT回0停机又高。精确脉宽含ARR+1/CCR边界，不能把duration参数宣称为零误差实测脉宽。
- 射球前startKick先CHG低，再启动对应计时器。IRQ收到结束事件才进入COOLDOWN，默认80ms后恢复IDLE；KICKING和COOLDOWN均禁充，也拒绝普通重复arm。

v13b图纸链：X1.6/7 -> SI8620 IC1A/B（非反相）-> FAN3229 IC8的-IN，+IN接12V -> OUT经10欧至IGBT门极。因此控制输入高让驱动输出低、IGBT关；控制输入低让驱动输出高、IGBT开。来源为原理图两页、EAGLE对应DIS1/DIS2网及onsemi数据手册：https://www.onsemi.com/download/data-sheet/pdf/fan3229t-d.pdf 。

SRC当前Core/Src/tim.c:458与misc.c:kick_pulse：PWM2/High，period636，CCR=636-ticks。以value1（ticks5）为例，启动后低约12.62ms，高100us，OPM回0后又低；**这与v13b要求相反**。初始及shooter_off使用HAL_TIM_PWM_Stop关闭CCxE，此时物理脚电平不是仅凭CNT/CCR可保证，且配置无内部上拉。

此前新增80ms充电冷却只约束CHG，不会自动把DIS改成高，也不能关闭错误极性下的IGBT。已有board_kicker_ready只检查CC1P/CC2P为0，不验证是否与v13b整条反相链兼容；宿主pulse测试也把高有效当预期，故通过不能证明v13b适配正确。

必要修正建议（本轮未实施）：
1. 两路DIS在GPIO接管、AF切换、待机、发射后、停止、HAL失败、故障和关机阶段都保证高；复位阶段还需核对主板/隔离器输入的外部偏置。
2. 按低有效重新设计脉冲建立/回零，核对TIM9共享ARR、两通道互斥、通道启停时电平；不能只翻CCER而沿用现有board_kicker_ready极性守卫。
3. 同步改普通射球与关机放电、时间换算/安全守卫/测试桩；以DIS高关闭及IGBT门极低为验收，不以HAL停止函数名为证据。

## 8. LT3751每组相关引脚由谁控制

| 引脚 | v13b连接/用途 | 固件是否驱动 |
| --- | --- | --- |
| 1 RVTRANS | R9=24.9k至变压器输入母线，检测输入 | 否，硬件电阻 |
| 2/4 UVLO1/2 | R6/R8=133k分别采主母线/12V，硬件欠压条件 | 否 |
| 3/5 OVLO1/2 | 图纸禁用过压门限的硬件接法 | 否，不是软件使能 |
| 6 FAULT | 此图未见引到X1或MCU的独立通道 | 否 |
| 7 DONE | X1.8，100nF对地；SRC PC13读取，TIGERs本地控制未用 | 只读，非输出 |
| 8 CHARGE | X1.5直接输入，R2=10k下拉 | 是，唯一直接充电请求 |
| 9 CLAMP | 接12V | 否 |
| 10 FB | 图纸接GND，使用该反激充电控制方式 | 否 |
| 11 CSN/12 CSP | R18=8mΩ采样链 | 否，由LT3751限流 |
| 13 VCC | 12V偏置电源 | 否 |
| 14 LVGATE | 图纸接GND | 否，不需要MCU激活 |
| 15 HVGATE | 经R14=0到Q1门极 | LT3751自己输出，不是MCU PWM |
| 16 RBG | R12=1k对地 | 否 |
| 18 RVOUT | R13=24.9k硬件输出目标设定链 | 否，软件200V目标与硬件上限不是同一参数 |
| 20 RDCM | R15=10k接变压器开关节点 | 否 |
| PAD/GND | 地 | 否 |

v13b注释标称欠压7.9V、硬件目标244V（图注tested250V），不作为本次实物实测结果。没有遗漏的MCU GPIO“升压PWM”或SPI寄存器写入。

## 9. 本地TIGERs源码本身的证据边界

shared/module/kicker.c:325用GPIOPinIsSet读取是否已充电；shared/hal/init_hal.h:111定义为读取GPIO->BSRR。该寄存器用于写置位/清零，不是持续输出/输入状态。若按常见read-as-zero行为，isCharging会一直假，进度基准每次重置，3s看门狗不能按设计触发。这是本地参考版本的静态缺陷/待硬件确认，不可归因于现役TIGERs，也不应移植到SRC。SRC读HAL_GPIO_ReadPin（IDR）保留正确的物理电平观察语义。

ST寄存器参考RM0433 GPIO章节；本轮官方PDF在线抓取受文件大小限制，没有声称从在线工具完整读完。ST社区解释作为辅助：https://community.st.com/stm32-mcus-products-25/the-hal-function-to-read-the-current-output-status-of-a-gpio-65676 。本结论主要基于本地宏及寄存器用途，不声称已运行构建TIGERs。

## 10. 本轮操作与限制

- 已阅读双方SYS/DEV/模块/底层GPIO、SPI、timer、电源管理与默认配置；读取并渲染v13b两页原理图、SRC主板图，用EAGLE网表核对X1及反相驱动连接。
- tmp/compare_kick_polarity.py根据两份生产源的OCM字段推演CNT0、比较点和OPM回零电平，确认参考高空闲/PWM1低脉冲与SRC PWM2高脉冲相反；这是寄存器模型推演，不是示波器实测。
- 更新本报告、BOOST_COMPARISON.md及AGENT.md；本轮未改固件行为、引脚、CubeMX、保护或参数，未构建/烧录。当前旧镜像不应继续用于v13b带高压射球自检，先修DIS关闭状态再验证。
- 用户CHG高3s证明该测点有使能窗口；仍未确认控制器脚8、电源轨、DIS和IGBT门极波形，也未独立测出真实电容电压。不能从NO RISE指定LT3751损坏或把所有原因归硬件。

## DONE/CHG初始化的明确对照（2026-10-20再核验）

| 阶段 | TIGERs指定目录实际源码 | SRC第77节之后 |
| --- | --- | --- |
| DONE | 无对应输入初始化或判读路径 | PC13输入上拉，充电故障门控使用 |
| CHG初态 | PG11在识板阶段为输入 | PC4从早期初始化起推挽低 |
| 板识别 | PG11弱下拉10ms读、弱上拉10ms读，利用板上10k下拉 | 无 |
| 切输出 | 先写PG11低，再推挽输出/无拉 | 先写PC4低，再推挽输出/无拉 |
| 首次20ms保持低 | 没有专门这项；识板两次10ms是另一回事 | 有，SRC自行增加 |
| DONE启动20ms观察 | 没有，因为不判DONE | 有，每次CHG低到高后重计 |
| 开始充电 | AUTO_CHARGE请求+测量/状态允许 | 自动请求+测量/状态允许 |
| 充电循环 | 事件驱动，等待超时约1ms | BoardUpdate约10ms |
| 停止条件 | ADC超时、温度、目标电压、kick/cooldown等 | 除上述还使用DONE/电池有效/quiet等 |

对应TIGERs证据：shared/module/kicker.c:194（任务）、:219（写低再切输出）、:322（充电）、:477（识板）、:510（ADC滤波）；main2019/robot_impl.c:452（模式请求）。**SRC新增的两个20ms不是TIGERs策略，也不是从其代码移植的参数。**
