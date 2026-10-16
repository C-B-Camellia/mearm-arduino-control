# 双板业务命令整合交付 2026-10-16

## 基线与交付边界

以本地未提交源码为基线，无整文件旧版覆盖。Uno分支 codex/interaction-patch-2026-10-15，HEAD 16307e039368ef54b9484bd1010ce72a7ddfe9e6；副板所在开发分支 codex/emergency-fallback-2026-10-13，HEAD e76bf95d869588739b9979ce5f805ed30fb53789。下面结果对应当前未提交工作树，不冒充上述HEAD本身的构建结果。没有Git提交/推送、烧录或EEPROM硬件操作。

基线：D:/Develop/mearm-arduino-control/exports/dual-board-refactor-baseline-20261016/，uno-before.zip和nano-before.zip含源码、Git差异、构建产物；原副板共享的开发根include/button_protocol.h未改动，另存nano-original-shared-button_protocol.h方便恢复。证据：同级dual-board-refactor-evidence/，包含本轮独立diff、源码SHA256、最终ELF/HEX、日志、节区/符号大小和实验B副本。ZIP不是现场EEPROM内容备份。

## 数据流与功能审计

副板消抖/短长按→本地模式映射→业务ID→8字节帧→Uno D10校验→dispatchAction→原业务API→唯一MotionTask→arm最终合法性验证→Servo。
USB pollSerial→executeLine→executeAppCommand→同一dispatchAction。USB独立64字节缓冲没有被D10覆写。

删除Uno onButtonPressed、AppAction::Button、BTN解析、旧B1文本串口兼容分支；原按键必要的Busy约束迁移至Pick/Play业务入口。PICK NEXT只在请求接受成功后推进A/B/C，副板不维护物体轮转。
删除获授权的LINE/N自定义坐标解析，固定预设位于src/app_controller.cpp drawPresets PROGMEM，AVR用memcpy_P取16字节局部副本。N四顶点顺序保持原样，固定直线(125,30)到(125,100)，N边界(115,-40)/(155,10)。motion.cpp本轮仅DONE标签DRAW SHAPE→DRAW N，没有改绘图执行器。

与备份逐文件比较确认：arm、hardware_config、joystick、kinematics、main、manual_control、path_planner、pick_place、pin_uart、recording、teaching、control_settings和platformio.ini未改。保留真实标定194/117、组合限位、Fast80、POLY500ms、CURVE1.20、落笔倍率/500ms、LINE/N其他顶点1000ms。保留PICK编辑与EEPROM、REC/PLAY、HOME、JOINT/CART、O/S/H/L、三轴指令、ORDER、三高度、全部预检及最终输出检查。

EEPROM仍为版本1，两个155字节槽+147字节编辑区，共457字节；CRC、代际选择、最后写提交标志机制未变。没有迁移/清空EEPROM，PICK各阶段独立四轴角度不恢复旧限制。

## 协议与方案实测

仍9600 baud、8N1、A5/type/epochLo/epochHi/sequence/payload/CRC8/5A，CRC前6字节多项式07。Hello=0保留；新Command=2，旧Press=1拒绝。保留epoch、sequence、重复过滤、超时与接收异常恢复。不加返回线路。两端include/business_commands.h和button_protocol.h逐字节一致；开发工程根include/button_protocol.h保持旧版，不用于本次副板。

| 测量方案 | Uno Flash | 静态SRAM | 最大帧 |
|---|---:|---:|---:|
| A：业务ID，最终生产 | 31330 | 1293 | 8 B |
| B：带长度/CRC/会话/序号文本，隔离实验 | 31616 | 1308 | 21 B |

B仅用于真实链接容量比较，未做完整产品验证，不是可烧录候选。A比B省286 Flash、15 SRAM，调用更直接。理论线速8字节8.33ms，21字节21.88ms，不代表实际ISR执行耗时。协议有意不兼容旧按钮type，必须两板配套更新，不能单独升级后继续用旧板程序。

## ID与共同业务入口

| ID | USB等价命令 | 原有API/行为 |
|---|---|---|
| 1 | PICK NEXT | 接受Pick后推进nextObject |
| 2 | REC | startRecording/stopRecording |
| 3 | PLAY | startPlayback |
| 4 | HOME | cancelAndReturnHome |
| 5 | TEACH START | beginTeach |
| 6 | TEACH DRAWZ | captureTeachHeight(true) |
| 7 | TEACH TEACHZ | captureTeachAlignment |
| 8 | TEACH TRAVELZ | captureTeachHeight(false) |
| 9 | TEACH MARKING | beginMarking |
| 10 | TEACH EXIT | exitTeach |
| 11 | MARK | markTeachPoint |
| 12 | UNDO | undoTeachPoint |
| 13 | CLEAR | clearTeachPoints |
| 14 | DRAW LINE | startPreset→startDrawLine |
| 15 | DRAW N | startPreset→startBasicShape(N) |
| 16 | DRAW POLY | startPolyline |
| 17 | DRAW CURVE | startCurve |
| 18 | HOLD | pauseMotion/holdArm |
| 19 | GO | resumeMotion/resumeArm |
| 20 | CANCEL | 取消录制/绘图回撤/cancelMotion |

21 Pick、22 RecordStart、23 RecordStop仅USB/内部使用，不接收为远程ID。非法ID拒绝。USB仍可用A/B/C、REC START/STOP、PICK SET/SHOW/SAVE/LOAD/SPEED/WAIT、ORDER等原指令，详见README。远程接受在Uno USB打印OK REMOTE，异步结果仍DONE/ERR；OLED不会收到这些反馈。

## 页面与映射（采用用户最新H4决定）

| 本地模式 | B1短按 | B2短按 | B3短按 | B4短按 | 其他长按（≥800ms） |
|---|---|---|---|---|---|
| NORMAL | PICK NEXT | REC | PLAY | HOME | H1→TEACH START/CAL；H2→BASIC |
| BASIC | DRAW LINE→DRAWING | DRAW N→DRAWING | 返回NORMAL | HOME | H2→NORMAL |
| CAL | DRAWZ | TEACHZ | TRAVELZ | TEACH EXIT→NORMAL | H1退出；H2 CLEAR；H3 MARKING→TEACH |
| TEACH | MARK | POLY→DRAWING | CURVE→DRAWING | UNDO | H1退出；H2 CLEAR→CAL |
| DRAWING | HOLD | GO | CANCEL | 无动作 | H2仅本地返回原BASIC/TEACH |

所有模式H4同时切换页面和映射：NORMAL→BASIC→CAL→TEACH→DRAWING→NORMAL。进入CAL发送TEACH START，进入TEACH发送TEACH MARKING，DRAWING回NORMAL发送TEACH EXIT；其他H4转换不发送动作。一次长按只触发一次，不同时触发短按；带长按功能的键短按在松开后发送。开机按住须先松开再按。

这是本地选择，不是Uno状态：发送MARKING后必须看PC的TEACH READY再MARK；绘图结束须看DONE，才用DRAWING H2返回。请求被拒或丢帧时，页面仍可能已经改变，不能把页面当成确认。任务运行中切页不绕过Uno BUSY、安全预检或限位。OLED右上L1..L5表示本地模式，!表示发送队列曾满；没有五点计数或成功状态。

## 资源与栈风险

| 目标 | 基线Flash/SRAM | 最终Flash/SRAM | 变化 |
|---|---|---|---|
| uno_nano_five_fast | 32210 / 1295 | 31330 / 1293 | -880 / -2 B |
| nano_oled | 11566 / 1085 | 12824 / 919 | +1258 / -166 B |
| nano（无屏） | 本轮未测旧版 | 3256 / 262 | 不作差值 |

Uno最终97.1% Flash、63.1%静态SRAM，余926 Flash、755静态外SRAM；OLED41.7%/44.9%。Uno .text31078/.data252/.bss1041；副板.text12658/.data166/.bss753。基线分别31956/254/1041及11216/350/735。Flash=.text+.data，SRAM=.data+.bss。

旧onButtonPressed符号318字节、executeDrawCommand448字节消失；新startPreset166字节。其余LTO内联、调用布局也变化，不能将符号消失全部解释成删除功能，精确总收益以节区为准。符号差异见*-symbol-delta.json和*-symbols.txt；Uno固件MAP在candidate/uno中（已生成）。

录制480字节、MotionTask104、USB输入64、D10环形32/帧8未扩大。16字节预设局部仅startPreset使用且noinline，避免带入CANCEL深层IK路径。面板保留128字节OLED页缓冲，每行22字节临时字符；页面660字节常量移入PROGMEM。没有动态容器、新全屏缓冲或新执行器。755字节不等于已证明栈安全；最深取消/回家/IK链仍需实机栈高水位与中断叠加测量。Native耗时不作16MHz WCET证据。

## 构建与测试

实际构建：pio run -e uno_nano_five_fast；开发根目录pio run -d nano -e nano_oled -e nano，全成功。首次沙箱拒绝PlatformIO缓存写权限，提升执行权限后成功，未修改工具链。日志为uno-final-build.log/nano-final-build.log。

- 面板无屏712、OLED1472检查通过：5模式全部短长按、800ms边界、>2秒单次、开机按住、抖动、时间回绕、模式切换中旧按键释放、满队列、重启epoch、序号回绕、OLED宽度/模拟刷新。
- remote_command_tests：2872通过。实际面板模拟输出的二进制帧送入真实Uno解码/调度，覆盖三高度→等待回中MARKING→READY→5点→UNDO→POLY→HOLD/GO/CANCEL→CLEAR/EXIT；全部20命令USB与远程状态结果对照；PICK/HOME/CLEAR坏CRC/重复帧不误动作；USB半条J3命令与远程GO交错不破坏输入；旧BTN/自定义DRAW拒绝；固定LINE/N预检/执行完成。
- sprint：1376673检查通过（包含数学、录制回放、EEPROM/抓放等原回归）；D10驱动3610+协议38通过。
- interaction541、teach_axis470、interaction_draw15008、pen_down3566、vertex_hold24325、parameter80 7558，以及fast70/fixed_speed/speed/minimal/field原相关回归通过，详细日志保留。不将历史未执行套件宣称通过。
- 软件模拟不能证明真实D10/Servo中断相容性、OLED I2C最坏耗时、机械运动质量或电源安全。本轮驱动与控制节拍无改动，仍需要实测。

## 最短人工验收与回滚

1. 保存当前PICK SHOW A/B/C输出；源码备份不包含实际EEPROM。停止任务、断电，断开板间线再进行烧录；副板USB与7.4V外部电源互斥。
2. 本轮没有上传。人工选择正确COM，先在Uno目录构建/上传uno_nano_five_fast，再在开发根用nano_oled；两板更新完之前不联线。不要加EEPROM擦除或熔丝参数，不上传容量实验B。
3. 断电后按原接线D1→D10、共地、分别供电、不互接5V。Uno USB115200运行STATUS；副板运行9600，烧录115200，二者不是同一速率。
4. 先在非运动条件看H4一按一页且映射对应、松开不发旧短按；查看PC的OK REMOTE/ERR。CAL的切换会发送TEACH START，TEACH切换会请求MARKING并可能自动定位，必须留足空间并完成高度配置。未经高度/点位确认不要连续轮转并假设只是看说明。
5. NORMAL验证B2准备/主动停止，长短互斥；PICK NEXT在忙/配置缺失时不推进物体。CAL设置3高度，MARKING等READY，MARK/UNDO后确认LIST；DRAWING测试HOLD/GO/CANCEL，DONE后H2返回。固定LINE/N仍需现场三高度合法才运行，任务成功不等于笔迹通过。
6. 混版会拒绝旧type，不依赖它继续演示。回退须配套使用baseline的两端HEX或源码备份重新编译，不执行reset/覆盖新的现场标定。回退源码采用本轮diff逆向人工核对，避免覆盖其他未提交改动。EEPROM格式未改，无需清空。

## 本轮修改文件


### uno

- `include/app_controller.h`
- `include/business_commands.h`
- `include/button_protocol.h`
- `src/app_controller.cpp`
- `src/button_link.cpp`
- `src/commands.cpp`
- `src/motion.cpp`
- `test/draw_t3/fast70_tests.cpp`
- `test/draw_t3/interaction_draw_tests.cpp`
- `test/draw_t3/parameter80_tests.cpp`
- `test/draw_t3/pen_down_tests.cpp`
- `test/draw_t3/speed_tests.cpp`
- `test/draw_t3/vertex_hold_tests.cpp`
- `test/native/interaction_tests.cpp`
- `test/native/nano_link_tests.cpp`
- `test/native/remote_command_tests.cpp`
- `test/native/run.ps1`
- `test/native/sprint_tests.cpp`

### nano

- `include/business_commands.h`
- `include/button_protocol.h`
- `include/local_display.h`
- `include/panel_mode.h`
- `src/local_display.cpp`
- `src/main.cpp`
- `test/nano_tests.cpp`
- `test/run.ps1`

另更新Uno README、新增本报告和副板README。Git其他既有未提交文件不归于本轮。详细diff见证据目录uno-task-only.diff和nano-task-only.diff。

## 候选HEX SHA256

- `candidate\nano\firmware.hex`：`7c9a4a500f799e1c29a534e991026049858493dbd93d8ff852411242e34d53e2`
- `candidate\uno\firmware.hex`：`f99f721c4a5dc6e2e5ebd6f16b79ca7648971aaccb297ad28389e6305d06810a`
