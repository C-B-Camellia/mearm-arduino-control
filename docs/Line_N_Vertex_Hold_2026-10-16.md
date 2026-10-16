# LINE / SHAPE N 顶点1000ms等待补丁

工作区：`D:/Develop/MeArm-Bringup-2026-10-12`。分支 `codex/interaction-patch-2026-10-15`，HEAD仍为 `16307e039368ef54b9484bd1010ce72a7ddfe9e6`，本轮与之前的本地参数/标定修改均未提交。未推送、未烧录、未操作硬件。

## 本轮结果

- DRAW LINE：落笔首点和末点，恰好2×1000ms。
- DRAW SHAPE N：落笔首点、两个中间拐点、末点，恰好4×1000ms。
- POLY：用户澄清后保留本地实际配置，三个内部顶点各500ms，不改为1000ms。
- CURVE：不加停留；Fast80与切线增益1.20原样保留。
- 其他SHAPE不增加停留；ABC独立八阶段等待不变。
- 关节规划段输出确认后才计时；末点仍保持落笔姿态，等待结束后才加载抬笔段。无位置传感器，这不是物理到点测量。

## 实现审计

`drawGoal()`、`strokeSteps()`、`startDrawing()`逐字未改：仍生成相同几何、相同HOME/via和夹爪准备、接近首点、下降落笔、笔迹分段、抬笔、回HOME。

`execute()`中普通几何样本不设置endpointQueued；只有drawCheckpoint要求的实际端点设置它。`updateMotion()`仅在 `endpointQueued && !hasPendingPose()` 时确认端点，并且仅在此处completedWaypoints递增一次、endpointQueued清零，然后设置等待。因此未被arm接受的pending输出不会启动等待；等待循环也不会重复递增或重启计时。

笔迹区间用既有计数表示：

- 首个落笔端点 `penDown = homeViaCount + 4 + recoveryLift`，recoveryLift在DRAW表示是否插入夹爪准备。
- 最后笔迹端点 `lastInk = totalWaypoints - homeViaCount - 2`。
- `fromStart = completedWaypoints - penDown`，`toEnd = lastInk - completedWaypoints`；两者非负表示落笔端点区间。绘图总点数已在startDrawing中限制≤32767，这里的有符号16位距离不会溢出。
- LINE、SHAPE N在此区间所有已确认的真实端点等待1000ms。
- POLY额外要求fromStart、toEnd都非零，所以仅P2/P3/P4；时长继续取MEARM_POLY_HOLD_MS=500。
- LINE与POLY入口原来就把未使用的shape字段设为ShapeId::N，可与N统一过滤；TRIANGLE/Z/V和CURVE不进入新增等待。

未新增MotionState或MotionTask字段；任务AVR大小仍104 B。pauseMotion/resumeMotion/cancelAndReturnHome逐字未改：Paused直接跳过等待扣减；GO重置lastUpdate，继续剩余时间；DRAW等待期间仍被识别为Drawing任务，CANCEL/B3沿用抬笔并受控HOME。若恢复路径失败，原有安全停止仍保留。

## Flash限制下保留的局部等价精简

基线32250 B；第一版直观实现32318 B，超限62 B。尝试日志完整保留。最终只保留以下局部等价整理：

1. 连续DrawLine..Curve枚举范围代替四个OR判断。
2. HOME..CURVE原来全部使用DEFAULT_MOVE_SPEED，合并同一速度选择；普通运动和TeachPosition继续automaticSpeed，PICK各阶段仍随后覆盖成EEPROM速度。
3. 端点处理块复用不变TaskKind；PICK等待与绘图等待为互斥else-if。
4. 复用笔迹首末相对距离，避免LINE/N和POLY各自重复计算范围。
5. 等待仍先以32位计算时间差、限制到MANUAL_MAX_DT_MS=40，再把已被限制的小增量存为uint16参与倒计时。未削弱延迟tick限制、时间回绕或暂停保护。
6. 先返回尚在等待的任务，再检查任务最终完成，等价消除重复的零等待判断。

未采纳的类型缓存/端点标志扩展等尝试已撤回；未删除日志、功能或安全检查，未改编译选项。源码差异以 [本轮独立diff](vertex_hold_evidence_2026-10-16/changes.patch) 为准，不包含任务开始前已有的194/117、Fast80或曲线增益修改。

## 完整真实构建

`pio run -e uno_nano_five_fast` SUCCESS，真实本地标定及D10全部启用。

| 资源 | 基线 | 候选 |
|---|---:|---:|
| Flash | 32250 / 32256 B | **32256 / 32256 B** |
| 静态SRAM | 1295 / 2048 B | **1295 / 2048 B** |

ELF：text=32002、data=254、bss=1041。Flash无剩余空间，753 B非静态区域仍需容纳运行栈/中断，未测栈高水位或16MHz最坏周期。

[完整最终构建日志](vertex_hold_evidence_2026-10-16/final-build.log)，[源码和固件校验清单](vertex_hold_evidence_2026-10-16/manifest.json)。
HEX为 `.pio/build/uno_nano_five_fast/firmware.hex`，SHA256：

`080d9e7d6ee652aff8a49b4e120c6cd82eb05cf450eb172fd10e366f155dde3e`

## 回归结果及指令序列对照

| 回归 | 结果与重点 |
|---|---|
| vertex_hold_tests | PASS 28131。LINE/N，J3初始85/40，0/1/2个HOME中间点；恰好2/4次1000ms；pending未输出时零等待；确认时姿态等于目标；全等待期间保持落笔；每一顶点分别HOLD/GO、CANCEL和暂停后BTN3；延迟tick上界、时间回绕；其他SHAPE零新增等待 |
| 完整Native sprint | PASS 1375311，含ABC、EEPROM、录制回放、HOLD/HOME、安全限位、摇杆和绘图 |
| fast70_tests（历史文件名） | PASS 6776，按当前Fast80/POLY500运行，LINE/N新停留，POLY/CURVE不变 |
| interaction_draw_tests | PASS 15017，有无夹爪准备后的输出/时间序列一致，暂停取消 |
| speed_tests | PASS 4645，节拍、关节速度、暂停、回绕和完成 |
| minimal_tests | PASS 2756，路径几何、Z与曲线过点/量化 |
| field_tests | PASS 3589，组合限位边界回放/HOME、N门禁和不可达拒绝 |
| interaction_tests | PASS 628，TEACH异步流程、取消及错误反馈 |
| parameter80_tests | PASS 7369，Fast80、POLY3×500ms、CURVE零停留、端点与1.20增益保护 |
| D10寄存器/协议 | PASS 3610+38 |

每个1000ms等待都在主循环逐tick推进，无delay。专项测试在等待消耗140ms后HOLD 2000ms，剩余860ms保持；GO后再消耗860ms。末点同样验证。取消恢复后GO不会复活旧DRAW。

从本轮baseline.zip导出独立源码重跑同一轨迹，与候选逐条比较真实整数输出（忽略LINE/N预期新增的时间）：

| 轨迹 | 总模拟执行时间ms 前→后 | 发生变化的Servo输出帧 | 对照结论 |
|---|---|---:|---|
| 20mm LINE | 3020→5020 | 133→133 | 目标序列完全相同，仅增加2s |
| SHAPE N | 5340→9340 | 219→219 | 目标序列完全相同，仅增加4s |
| 五点POLY | 6000→6000 | 199→199 | 目标序列、时间、3×500ms全部相同 |
| 五点CURVE | 4660→4660 | 204→204 | 目标序列、时间全部相同，零停留 |

[逐项对照](vertex_hold_evidence_2026-10-16/trace-comparison.json)。这些是虚拟20ms时钟的指令结果，不是实物位置和运动时间测量。历史draw_t3_tests的固定高度扫描reversals>0断言在上轮补丁前后均失败，本轮未改标定/该断言，也未拿它宣称全部历史套件全绿；本轮相关套件如表。

## 修改范围、恢复与实机复测

本轮生产仅 `src/motion.cpp`；测试修改fast70_tests、interaction_draw_tests的等待预期，并新增vertex_hold_tests；补充本报告、README提示和证据。

已逐字校验保护文件不变：platformio.ini（POLY500）、control_settings.h（Fast80）、path_planner.cpp（增益1.20）、kinematics.cpp（194/117）、arm/joystick/manual/recording/pick/UART及hardware_config。

补丁前完整源码/测试/配置/固件快照：
`D:\Develop\MeArm-Bringup-2026-10-12\.pio\vertex-hold-20261016-144854\baseline.zip`

本轮回退可从该ZIP选择性恢复motion.cpp及本轮修改的两个旧测试；保留其他最新本地标定。不要整体覆盖随后可能新增的现场改动。原有分支/HEAD未移动，未执行提交或破坏性Git操作。旧ZIP/HEX不会被自动更新成本候选；候选另存同目录candidate.hex。

最短实机复测（由用户批准后执行）：使用已验证纸面与安全路径，LINE观察落笔首点/末点各约1s，N观察四顶点各约1s；在首点、末点等待时各做HOLD/GO，另一次CANCEL确认安全抬笔HOME；再跑原POLY/CURVE对照。固定等待只能保证指令保持，不能保证实际伺服反馈到位；留意末点停笔渗墨、机械受力和笔纸摩擦。代理未上传、未操作实体机械臂，等待用户确认。
