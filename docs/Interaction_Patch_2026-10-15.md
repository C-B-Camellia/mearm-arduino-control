# 阶段二交互补丁交付（2026-10-15）

## 身份与边界

- 工作区：`D:\Develop\MeArm-Bringup-2026-10-12`。
- 独立分支：`codex/interaction-patch-2026-10-15`。
- 原分支保留：`codex/r2-field-fixes-2026-10-14`，基线 `13250427469e9b4d6665816d5d502fdb9de6ea1f`。已核对其实际包含 Fast70 生产提交 `05b1c75edce7cc57bfcfd31fcabf67e182dc0a72`，不是假定 Fast35/70 状态。
- 本补丁最终生产源码：`103290642fbf74c40d158c05b020f208c0f7db34`。后续交付文档提交不改生产代码。源文件及HEX/ELF SHA见 [manifest](interaction_evidence/manifest.json)。
- 开始时无未提交生产修改；用户的未跟踪 `MeArm_Defense_Source.zip` 原样保留、未加入Git。
- 未修改开发工程，未上传、未操作硬件、未进入CORE-FROZEN。

## 分项提交与完整 D10 资源

所有数据均来自同一真实生产标定与 `uno_nano_five_fast` 链接结果，不是普通uno或合成标定结果。

| 项目 | commit | Flash B | 相对上一项 | 静态SRAM B/变化 | 构建证据 |
|---|---|---:|---:|---|---|
| Fast70基线 | `13250427469e9b4d6665816d5d502fdb9de6ea1f` | 31984 | — | 1295 / — | [0](interaction_evidence/0-baseline.log) |
| 1 删除HELP | `e747e79db25921135180410b3616b5fef220b68f` | 31782 | -202 | 1295 / 0 | [1](interaction_evidence/1-no-help.log) |
| 2 S=85 | `dd8bea9dd2b275c8a3bfea98cdcb3cf20ef90373` | 31782 | 0 | 1295 / 0 | [2](interaction_evidence/2-s85.log) |
| 3 命令上下文ACK/DONE | `4c89471289b3184f28673d209a27e7bf7eb503cc` | 31928 | +146 | 1295 / 0 | [3](interaction_evidence/3-context.log) |
| 4 短原因码 | `f2d5965b90de6510c5ab7da6eb0c7615dd1fc9cf` | 31908 | -20 | 1295 / 0 | [4](interaction_evidence/4-errors.log) |
| 5 MARKING异步定位 | `98605b49c78ddb458551897a9036b6b1661ef2e3` | 32254 | +346 | 1295 / 0 | [5](interaction_evidence/5-teach-first.log) |
| 6 DRAW准备及必要输出压缩 | `103290642fbf74c40d158c05b020f208c0f7db34` | 32254 | 0 | 1295 / 0 | [最终完整日志](interaction_evidence/6-final-build.log) |

净增Flash270 B；最终只余 **2 B Flash**。实际ELF `.text=32000`、`.data=254`、`.bss=1041`；Flash为text+data，静态SRAM为data+bss。753 B差额尚需栈/中断使用，未测AVR高水位，不能声称运行时栈安全已验证。

第6项第一次32354 B（超98），压缩后32284 B（仍超28），最终32254 B。尝试日志完整保留。允许范围内的调整：复用原串口缓冲恢复分隔符输出ACK；移除开机横幅、重复的RUNLINE预检失败句（仍有PATH ERR/ERR）；将 `PLAY timeScale=` 缩为 `PLAY x=`；移除不可能完成的None任务显示名称。没有删除功能、精简安全检查、缩减录制或关闭D10。`6-native.log`为早期非Fast辅助运行，最终Fast结果以`6-sprint_tests.cpp.log`为准。

MotionTask AVR大小仍为104 B（编译断言）。不增加输入副本、待命令字符串或独立执行器。TEACH通过现有MotionTask增加Waiting状态；DRAW复用任务已有布尔成员，按TaskKind区分HOME恢复抬笔/DRAW准备。TEACH回中分支在主循环motion更新中调用现有Cartesian规划；没有把IK新增到PC BTN3取消的解析调用链。本轮未测16MHz最坏循环时间或ISR抖动，不拿Native耗时冒充实机数据。

## 行为与代码证据

### 1–4：命令与反馈

`commands.cpp::result`复用64 B串口缓冲打印命令；`motion.cpp::finish`使用紧凑PROGMEM名称表。查询输出与录制/PICK保存专用生命周期保留。通用关节完成为DONE MOVE，PICK为DONE PICK；DRAW类型有独立DONE。没有为保存原命令新增缓冲。

S引用`control_settings.h::kPenGripAngle=85`，O仍引用`hardwareConfig.gripOpen=40`。物理配置gripClosed=90未改，PICK每阶段角度仍独立读取EEPROM。

错误原枚举0..10保留，新增11..16对应T01/T02/T03/D01/P01/P02。`control_error.cpp`为固定3字符表；精确映射、触发条件及操作建议在 [README错误表](../README.md#错误码速查)。专用码不会替代更早失败的通用检查。无法精确归因时保留E03/E05/E06/E07等兜底。

仓库测试没有主控响应驱动的外部上位机程序；活动测试已调整新响应断言。历史`test/native/test_main.cpp`原本已非当前入口，未为过期字符串断言改写。Nano→Uno单向帧不依赖PC的OK文本，实体按钮仍为OK BTN。用户自有上位机若精确匹配旧OK/长错误字符串，需要按README迁移。

### 5：MARKING

`teaching.cpp::beginMarking`先验证活动设置及drawZ<teachZ<=travelZ，再调用`startTeachPosition`。MotionTask占用等待，手动控制不抢占。`updateMotion`在控制tick读回中状态，保持当前接受姿态的FK XY并设置teachZ，交给原`startCartesianMove`/预检/输出。

`finish`仅成功时调用`completeMarking`输出READY。错误则HOLD、不进入Mark。`exitTeach`取消TeachPosition；B4对此任务明确退出；CANCEL/HOME重置同一个任务，无残留请求。HOLD/GO仅暂停继续。已有点和高度不清除。MOVE完成是软件指令完成，不是传感器确认。

### 6：DRAW准备

`startDrawing`仍验证高度、HOME/via；其夹爪需保持85。当前J3非85时增加首个准备waypoint。`drawGoal`该点只改变J3，原绘图索引顺延；`execute`和POLY顶点保持索引同步顺延，速度常量不改。

原整条预检包含准备段，失败不会先夹爪。任务全过程保持Drawing所有权，因此HOLD/GO/CANCEL/B1/B2/B3全部复用原路径。J3已85时无额外waypoint。取消准备仍沿用原绘图抬笔HOME（也受完整安全检查）；没有自动“笔已夹牢”判定。ABC结构、EEPROM和阶段抓放角完全未改。

## 回归证据（针对性断言，而非只看总数）

| 测试 | 关键断言 | 结果/日志 |
|---|---|---|
| interaction_tests | 非回中等待零输出→回中→保持XY/teachZ→READY→MARK；EXIT/CANCEL/B4无延迟复活；HOLD/GO；移动中退出；不可达时零输出/保留5点；T01/T02/T03；S/O；HELP/FEED拒绝 | 633，[日志](interaction_evidence/6-interaction_tests.cpp.log) |
| interaction_draw_tests | LINE/SHAPE/POLY/CURVE，J3=85和40两路；准备只改变J3；准备后每个20ms姿态、waypoint、保持时间均相同；暂停无输出、继续完成；CANCEL及BTN3不复活旧DRAW；整条预检失败连夹爪也不输出；D01 | 12865，[日志](interaction_evidence/6-draw-prep.log) |
| sprint_tests（实际Fast开关） | 位置/量化安全、二维/三维、严格解析、HOLD、摇杆、录制多场景/误差、回放、HOME、ABC八阶段、EEPROM损坏/断写恢复、点序/UNDO、绘图与故障恢复 | 1375442，[日志](interaction_evidence/6-sprint_tests.cpp.log) |
| teach_axis_tests | ADC映射、TEACH前后XYZ、J3摇杆冻结/显式调试、EXIT恢复、三高度排序与拒绝 | 470，[日志](interaction_evidence/6-teach_axis_tests.cpp.log) |
| D10寄存器模型 + PC/协议 | PB2/PCINT0、采样、正确/CRC坏帧、同步、序号、溢出、真实PC loop中的STATUS | 3610 + 38，[日志](interaction_evidence/6-d10.log) |
| field_tests | 边界组合角回放/HOME、N退化、曲线预检故障 | 3523，[日志](interaction_evidence/6-field_tests.cpp.log) |
| fast70_tests | 必经点、POLY三个80ms保持、CURVE零保持、按钮暂停/取消、最终HOME | 5718，[日志](interaction_evidence/6-fast70_tests.cpp.log) |
| minimal/speed/draw_t3/teach_grip | 路径/Z、规划节拍、量化及历史高度诊断、TEACH夹笔 | 2512 / 4491 / 5601 / 104，见同目录各suite日志 |

另外对原Fast70归档逐行比较OUT/KNOT/HOLD/METRIC/INTERRUPT：**730条完全一致**，见 [对照结果](interaction_evidence/6-fast70-equivalence.txt)。该对照起始J3=85；新增J3=40情况由上表独立比较准备后全序列。

模拟20ms时钟的原Fast70与本补丁（起始J3=85）均为：

| 轨迹 | 总规划执行ms | 描线ms | 发生变化的输出帧 | 最大相邻角变化 | 顶点保持 |
|---|---:|---:|---:|---:|---|
| 20mm LINE | 2900 | 580 | 111 | 2° | 无 |
| N | 8220 | 5420 | 231 | 2° | 原图形端点 |
| 五点POLY | 4600 | 2360 | 182 | 2° | 3×80ms |
| 五点CURVE | 4480 | 2240 | 185 | 2° | 无额外保持 |

五点采用用户已给的106.56/26.37、147.29/22.12、159.81/-17.72、129.64/-29.16、107.31/-11.54顺序；测试高度82.53/86.71/94.29来自历史故障样本。它们只用于测试，没有写入生产纸面配置。不是现场耗时或轨迹精度测量。

## 修改文件

生产：`include/{command_text,control_error,control_settings,motion,teaching}.h`，`src/{app_controller,command_text,commands,control_error,main,motion,teaching}.cpp`。
测试：`test/native/{sprint_tests,teach_axis_tests,interaction_tests}.cpp`，`test/draw_t3/{draw_t3_tests,speed_tests,teach_grip_tests,interaction_draw_tests}.cpp`；以及逐项日志、manifest和README/本报告。

`hardware_config.cpp`、`arm.cpp`、`kinematics.cpp`、`path_planner.cpp`、`manual_control.cpp`、`joystick.cpp`、`recording.cpp`、`pick_place.cpp`、D10驱动/协议与`platformio.ini`相对基线不变。

## HEX、烧录与最短复测

完整产物：`.pio/build/uno_nano_five_fast/firmware.hex`。

SHA256：`f9d492f89ebdc182e0dd7bb8cf2c3e81fed09440443f5649fa7edb02b6d650c0`。

仅供用户确认后手动执行，代理未执行：

```powershell
Set-Location D:\Develop\MeArm-Bringup-2026-10-12
pio run -e uno_nano_five_fast
Get-FileHash .pio/build/uno_nano_five_fast/firmware.hex -Algorithm SHA256
pio run -e uno_nano_five_fast -t upload --upload-port COM实际端口
pio device monitor -b 115200 --port COM实际端口
```

烧录/复位会走原有舵机启动输出，先按已验证的供电、断线和机械清场流程处理；不能把“只发STATUS”理解为复位不会输出。预期不再有开机横幅。

1. 主动发送STATUS，核对mode、enabled=15、held、J及错误。`HELP`/`FEED 35`应只报ERR E05。确认O目标40、S目标85且OK带命令；这些是实际运动测试，先无笔/无负载核验夹爪。
2. `TEACH START`，在真实纸面与安全位置依次捕获DRAWZ/TEACHZ/TRAVELZ；STATUS核对顺序。未完成真实高度前不要复制本报告测试高度。
3. 摇杆轻偏时发`TEACH MARKING`：看到TEACH WAIT及OK，后续不再手动移动；回中看到MOVING、PATH VALID、READY。检查XY保持、自动到对点高度，之后MARK才成功。
4. 再发MARKING并保持偏杆，用TEACH EXIT或CANCEL取消；回中、确认后GO，不能延迟启动。等待和移动状态都各做一次。点集LIST不丢失。
5. 在已验证可画路径和笔可受控装夹的情况下，分别J3=85和合法非85值运行同一DRAW：前者无准备段；后者先夹爪至85再走原路径。准备途中HOLD应停止、GO继续；另一次CANCEL/BTN3应只抬笔HOME，不恢复旧DRAW。不要将自动闭爪等同笔已固定。
6. 复用已验证的五点、顺序和纸面，POLY/CURVE各一次，检查速度、80ms顶点保持和曲线连续性不变；最后短录制B2开始/结束、B3回放、B4 HOME和Nano按键回归。

失败时保存完整命令/ERR/PATH ERR/STATUS及现象。预检失败应无DRAW准备动作；若发生意外移动、夹持异常或未能停止，立即按现场既有停止/断电流程处理。本轮没有物理反馈可替代人工判断。

## 回退

原分支和基线未改。不要在有新标定/未提交修改时强制切换。可使用一个**不存在的新目录**单独检出基线并构建（不覆盖当前目录）：

```powershell
git worktree add --detach ..\MeArm-Interaction-Rollback-2026-10-15 13250427469e9b4d6665816d5d502fdb9de6ea1f
Set-Location ..\MeArm-Interaction-Rollback-2026-10-15
pio run -e uno_nano_five_fast
```

若目录已存在，先检查，勿覆盖。基线固件保留旧HELP、S=90、MARKING前置条件和DRAW夹爪拒绝行为。EEPROM格式本轮未变；仍应保留现场原始标定记录。

六项软件已完成并通过完整D10构建/所列回归。未完成项为新交互的实机验收、物理到点/夹持验证、AVR最坏时序与栈高水位测量；Flash仅2 B余量是明确限制，不能据此继续堆功能。本轮到此停止，等待用户决定烧录。
