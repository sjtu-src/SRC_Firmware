> 2026-10-20 后续重要修订：用户已确认CHG高约3s、射球板为TIGERs Kicking Device v13b。逐原理图/底层源码发现DIS1/2应高关闭、低射球，SRC当前PWM2/High脉冲及结束回零低与该板反相链冲突。此前仅检查CCER高极性的适配判断不充分。最新逐引脚审计见 [BOOST_PIN_AUDIT.md](BOOST_PIN_AUDIT.md)。下文前半为较早阶段分析，禁充显示/冷却已实现，但DIS极性修正尚未实施。
# 功率板升压与 TIGERs 本地固件对比（2026-10-20）

基线：SRC d80b9f3501492870f11270f9beb8e1cf79a3d6bb，开始工作区干净。参考 E:/SRC/Tigers/Firmware-master/Firmware-master（README 面向 v2020；无可靠提交号，不代表现役比赛固件）。本轮为分析，不修改固件行为。

用户本轮实测：BAT 22.5V，CAP 0.01V，PC4/CHG_EN 一直低。尚未确认测量仪器、时间分辨率、运行镜像、PC13 和完整状态快照。历史22.2V/0.1V不再作为本轮读数。

## 当前结论

SRC 已有自动升压路径，不缺 TIGERs 的某个启动命令。board_monitor_update 每10ms执行，健康测量、温度正常、无保护且电容低于175V时应输出PC4高。22.5V高于当前6S的19.8V禁充阈值，因此不能仅归因于当前电池电压低。

本轮以生产 board_monitor.c 和现有HAL桩复现：22.5V、约25°C、近零电容时CHG_EN=1；无法从现有读数确定实机是哪项门控阻止。0.01V模拟输入因ADC量化成为0V，不冒称复现了硬件精确0.01V。

## 逐项对照与修改建议

| 项目 | SRC当前 | TIGERs本地 | 判断/建议 |
| --- | --- | --- | --- |
| 充电请求 | 无独立请求；条件满足自动充电 | AUTO_CHARGE模式，上层系统使能和比赛包控制 | 属于产品策略差异，不是SRC缺使能。以后加入显式充电请求，需同时适配上位机和自检；此次无需为开机升压照搬 |
| 输出 | PC4推挽，高允许，初始化低 | PG11，高允许；识别功率板后设输出 | 未发现应翻转极性的源码依据；不要照搬引脚号或弱上/下拉识别，先核对SRC实际接线 |
| 电池门控 | INA226有效且>19.8V；100ms超时；无效/低压5s锁存quiet | chargeControl不直接检查INA226标志；整机power_control另行管理电源状态 | 保留有效性/欠压保护；必须显示quiet及首次触发原因/时间。自动解除锁存是否允许属于行为修改，需另定恢复策略 |
| DONE/FAULT | PC13低有效，充电中且CAP<175V立即故障，等待30s | kicker.c无这一独立输入门控，以电压/进度为主 | 优先核实PC13究竟接DONE还是FAULT及有效电平；显式区分已充满、硬故障、无进度，按器件时序处理启动与锁存，不能直接删保护 |
| ADC | SPI2 mode3、8bit/MSB；每通道16样本，弃前2个，至少4个有效；额外温度合理性检查 | 同样通道/弃样/换算；约1ms独立任务 | 没发现通道或字节序明显迁移错误。SRC一次SPI失败会立即禁止充电；应增加失败状态、两通道原始均值/有效样本数与时间戳诊断 |
| 电容反馈 | 未作跨帧滤波，目标180V、5V回差 | 快/慢EMA（0.8/0.98），默认200V、5V回差 | 加滤波时按SRC实际10ms周期重算时间常数；保留原始过压检查。不要把200/220V参考值直接改入SRC |
| 过压 | 当前读数>=180V停充 | 目标停充外，慢滤波>260V有独立临界检查/日志 | 建议独立过压原因和异常测量检查；阈值须按SRC元件额定值/标定确定，不能复制260V（SRC251.42V标称ADC满量程本身不足以测到该值） |
| 温度 | >=60°C停、<=55°C恢复；热状态为函数静态变量 | >60°C停、<55°C恢复，过热错误可读 | 门限基本一致。SRC应公开热锁存状态，不能只显示瞬时温度，否则55–60°C回差带会误判 |
| 充电进度 | 每累计增加5V重置3s计时，失败等30s | 同样3s/5V/30s，但监测基准和错误清除细节不同 | 已有同类机制；增加无进度故障、起始电压、剩余重试时间诊断，无需先延长超时掩盖问题 |
| 踢球停充 | TIM9 CEN期间禁止，结束后下次BoardUpdate可恢复 | KICKING和COOLDOWN全段停充 | 明确需补齐射球后冷却状态/时间戳及充电门控；冷却时长按SRC硬件确认，非当前未充电时PC4低的直接解释 |
| 状态可见性 | board_monitor_get有部分状态，普通OLED只有电压；hot/剩余重试/原因无公开完整快照 | kicker status显示错误、温度、电压、状态、模式 | 第一优先修改：提供同时刻禁充原因位图、请求/实际电平、输入、测量年龄及锁存来源；允许多原因并存 |

源码证据：Core/Src/board_monitor.c 的 kicker_read、update_power_protection、board_monitor_update、board_actuators_disabled；Core/Src/freertos.c 的 Update_Board；Core/Src/misc.c 的 kick_pulse/start_kick；Core/Src/gpio.c 与 spi.c；Core/Inc/cfg.h。参考 src/shared/module/kicker.c 的 chargeControl/startKick/adcKickerUpdate/status，src/shared/drv/adc_kicker.c，src/shared/module/power_control.c，src/main2019/tiger_bot.c、robot_impl.c 和 src/robot/robot.c。

## 两个已经复现的候选路径

1. 启动INA226读取失败超过5s后quiet锁存。随后读取恢复且BAT=22.5V、valid=1，仍quiet=1且CHG=0。当前BAT显示正常不能排除此路径。该锁存也会禁止四轮和吸球，所以若同次上电这些执行器确实仍可运行，应降低它的嫌疑。
2. PC13持续低：CHG拉高后下一次10ms监控触发charge_fault并拉低，等待30s后重试，再约10ms后拉低。若用户用万用表观察，可能看起来始终低；若已用示波器覆盖启动和完整重试周期仍完全无高脉冲，则单独这一条件不足以解释，优先查其它门控或输出配置/外部钳位。

下一步应取得一个同时刻快照：battery_valid/kicker_valid、temperature及hot、quiet/shutdown、charge_fault及重试剩余时间、kick_done/PC13、TIM9 CR1.CEN，以及PC4 IDR/ODR与GPIO模式。这样才能判定需要修哪一项，不强制拉高或屏蔽保护。

## 验证与限制

- 当前生产代码的临时复现脚本：tmp/boost_review.py；正常22.5V、DONE持续低重试、采样恢复后quiet锁存三个场景通过。
- 4S/6S各12项与电源/充电/脉冲/关机放电相关既有用例通过，共24项。
- 完整 tests/test_board_migration.py 在 drib_diag 失败：测试仍预期参考55%输出，而用户当前 DRIBBLER_MAX_DUTY=0.15。未为通过测试改回用户参数，也未把完整套件声称为通过。此测试适配问题与本次升压结论分开记录。
- 未修改生产固件、CubeMX、协议、目标电压或用户15%吸球上限；未Keil构建、烧录或实车测量。宿主通过不证明物理PC4已输出或功率级正常。
- BOARD_MIGRATION.md中吸球PI和早期关机叙述为历史实现；当前以源码与AGENT.md后续节为准，本轮不扩展吸球任务。

## 2026-10-20 已实现：OLED禁充原因与射球后停充

用户确认 PC13/KICK_ERR 实际接 LT3751 DONE，低有效。保留生成的引脚名和原有低电容DONE/无进度30s重试保护，不再把该引脚称为独立FAULT输入。

新增 OLED 状态行位于 y=43..50，各模式均显示；四轮行压缩至 y=10/18/26/34，底部频率/车号/模式栏保留。多个原因每1.5s轮换：

| 显示 | 含义 |
| --- | --- |
| CHG:ON | 当前允许充电 |
| CALIB | 标定使能未确认 |
| CAP ADC | 电容/温度ADC读取无效 |
| BAT DATA | 电池监测数据无效 |
| BAT LOW | 电池电压未超过保护阈值 |
| HOT | 过热锁存（降到55°C或以下恢复） |
| DONE LOW Ns | 低电容充电时DONE有效，等待N秒重试；不等同于已经确定芯片损坏 |
| NO RISE Ns | 充电电压进度不足，等待N秒重试 |
| KICK | 射球脉冲运行中 |
| COOLDOWN | 射球后充电冷却 |
| QUIET | 电源静默保护已锁存，当前电压恢复也不会自动解除 |
| SHUTDOWN | 已请求关机（正常界面让位于既有关机专页） |
| FULL/HYST | 电容已达目标或处于停止后的电压回差带 |
| CHG:WAIT | 尚未有完整状态的过渡情况 |

board_status_t新增charge_inhibit位图、charge_fault_reason、charge_retry_ms和charge_cooldown_ms；多项可同时存在，不只显示第一个原因。冷却宏KICKER_CHARGE_COOLDOWN_MS=80ms，依据本地TIGERs默认值；不是本板实测的最佳冷却时间。成功启动射球后记录事件，BoardUpdate看到TIM9停止才开始80ms冷却；因此不漏掉监控两次采样之间已结束的短脉冲，实际从物理脉冲结束到恢复可多一个监控周期/调度延迟。新射球重启冷却事件；这里只增加充电门控，未增加普通射球命令的最短间隔。

本轮构建0错误0警告，Code54622、RO-data5118、RW-data500、ZI-data29932，日志tmp/charge_inhibit_build.log，正常AXF/HEX已更新。tests/test_charge_inhibit.py在4S/6S各10项新增及12项原有相关用例通过（44项）；test_charge_oled.py验证实际显示块的全部标签、倒计时、轮换和宽度；音乐/四轮启动回归通过，git diff --check通过。完整旧板级套件的55%吸球预期与现有15%不匹配问题仍未改动。

未烧录、未实车验证；宿主OLED桩只验证文本及坐标约束，不代替实屏观察。上电后先记录CHG状态行，多个原因应观察至少一个完整轮换周期。
