# N原方向与局部落笔倍率补丁

日期2026-10-16，工作区 `D:/Develop/MeArm-Bringup-2026-10-12`。分支 `codex/interaction-patch-2026-10-15`，HEAD仍 `16307e039368ef54b9484bd1010ce72a7ddfe9e6`。本轮和此前本地改动均未提交，未推送、未上传、未操作硬件。

## 用户澄清与最终行为

1. N恢复原始顺序。`DRAW SHAPE N 115 -40 155 10`：
   `(115,-40) → (115,10) → (155,-40) → (155,10)`。
2. 仅落笔阶段的空间速度目标乘1.50：25→37.5mm/s；抬笔仍25mm/s。纸面描线仍Fast80，曲线切线增益1.20不变，控制tick20ms不变。
3. 经用户澄清，所有保留DRAW在落笔输出确认后统一停500ms。LINE最后一点仍1000ms；N其余三个顶点仍各1000ms；POLY三个内部顶点仍各500ms；CURVE除落笔后的500ms，不增加逐点停留。
4. 容量不足后用户明确选择仅删除TRIANGLE/Z/V。当前保留DRAW LINE、DRAW SHAPE N、DRAW POLY、DRAW CURVE；退役命令返回ERR E05，不执行成N。ShapeId中旧枚举值保留供拒绝测试使用，不再有对应几何/执行功能。

## 最小实现与限速公式

生产修改4个文件：

- `src/path_planner.cpp`：去掉反序N及退役图形共表。原始N编码0/4/2/6对应系数(0,0)/(0,1)/(1,0)/(1,1)，用索引生成相同0/1系数；坐标公式 `lo + coefficient * (hi-lo)`逐字保留，采样和插值精度不变。40组边界矩形×4点与原始公式逐位比较通过。
- `include/control_settings.h`：新增唯一局部倍率 `kPenDownSpeedScale=1.50f`，基础转换速度仍25。
- `src/motion.cpp`：在已有penDown/penUp索引判定内，仅index==penDown时选37.5mm/s，其他阶段照旧；同一MotionTask在该路点输出确认后设置dwellRemaining=500。未改变HOLD/GO/CANCEL实现。只剩N后内部SHAPE固定4点；公共入口仍拒绝退役类型。
- `src/app_controller.cpp`：SHAPE仅接受N；保留LINE/POLY/CURVE解析与业务。

[本轮具体diff](pen_down_evidence_2026-10-16/changes.patch)基于本轮开始的真实工作树，不包含此前Fast80、POLY500、194/117标定等已有改动。

当前DRAW的automaticSpeed()不决定落笔关节限速：绘图阶段仍固定DEFAULT_MOVE_SPEED=60°/s。设落笔高度差d(mm)，单段最大关节角差Δq(°)：

```text
T_joint = ceil(Δq × 1000 / 60)
T_space_before = ceil(d × 1000 / 25)
T_space_after  = ceil(d × 1000 / 37.5)
T_down = max(10ms, T_joint, T_space_after)
```

只提高空间目标，没有直接缩短总duration，没有改变60°/s上限。若T_joint主导，则实际倍率可能小于1.50甚至没有提速；其余速度/时间策略不变。HOME、空中接近、夹爪准备、RETURN仍原策略；普通串口/TEACH的automaticSpeed、ABC分阶段EEPROM速度和回放时间轴也不变。

四类DRAW的当前测试高度差11.76mm：旧空间471ms、新空间314ms；关节约215–217ms（指定N命令194ms），因此计划471→314ms，倍率1.50。实际20ms输出节拍会量化该时间，硬件跟随速度不由这些数字保证。

使用当前真实标定扫描得到1097组有效垂直端点样本，本次没有找到关节约束主导的样本（关节时长/新空间时长最大0.833333）。这是有限网格证据，不是对所有姿态、路径及标定的全局证明；保留max关节约束仍然必要。

## 真实构建与容量

目标 `pio run -e uno_nano_five_fast` SUCCESS。

| 配置 | Flash B | 静态SRAM B |
|---|---:|---:|
| 本轮真实基线（N反序） | 32256 / 32256 | 1295 / 2048 |
| 初次倍率及等待补丁 | 32306，超50 | 1295 |
| 共用落笔等待范围后 | 32296，超40 | 1295 |
| 用户授权删三图形后，直接实现 | 32316，仍超60 | 1295 |
| 删除通用SHAPE计数/解码冗余，保留N原公式 | **32252 / 32256，剩4 B** | **1295 / 2048** |

没有因为源码变少就假定节省Flash；以上为真实链接。只剩N后消除无用通用顶点表和系数转换，才容纳补丁。未删除安全检查、未改编译选项、未降低核心功能配置。

ELF：text=31998、data=254、bss=1041；MotionTask仍104 B，无新状态机/持久字段。753 B静态之外空间仍需栈/中断，未测高水位。

[完整构建日志](pen_down_evidence_2026-10-16/final-build.log)，[源码/固件身份清单](pen_down_evidence_2026-10-16/manifest.json)。
HEX：`.pio/build/uno_nano_five_fast/firmware.hex`。
SHA256：`e0c100ef663bda5b12403c73bdf5ebee76c6f016a84ea1e9adcf23f658cac20a`。

## 测试范围

| 套件 | 结果与覆盖 |
|---|---|
| pen_down_tests | PASS 3566。N原顺序/原公式逐位等价；用户指定N命令完整执行；退役命令明确拒绝；每类DRAW落笔倍率；所有非落笔阶段原时间公式；每帧float关节推进≤60°/s对应增量；输出确认前不计等待、确认后500ms；1097有效样本限速审查 |
| fixed_speed_tests | PASS 25663。逐段验证max(原最小时长、原关节时长、空间时长)，仅落笔用37.5；抬笔25、描线80、空中运动仍原公式 |
| 完整Native sprint | PASS 1375240。运动安全、二维/三维、手动、按钮、ABC/EEPROM、录制回放、HOME、示教及绘图；退役图形改为明确拒绝测试 |
| vertex_hold_tests | PASS 24325。首点500、其他LINE/N顶点1000，J3准备/0–2个HOME中间点，pending确认、每个等待点暂停恢复/取消、时间回绕 |
| fast70_tests（历史文件名） | PASS 6752。Fast80、所有必经点、等待数量与暂停取消 |
| interaction_draw_tests | PASS 14782。有/无夹爪准备后等价输出、暂停取消 |
| parameter80_tests | PASS 7558。80mm/s描线、POLY内部3×500ms、CURVE无逐点停留、1.20切线增益 |
| interaction_tests | PASS 628。TEACH异步及错误反馈 |
| speed/minimal/field | PASS 4524 / 2741 / 3556。时钟/几何/Z/量化、不可达与边界HOME |
| D10寄存器/PC协议 | PASS 3610+38 |

Native计数变化包含已授权退役图形和新等待语义，不以总数代替具体断言。历史draw_t3_tests的reversals>0旧诊断在先前194/117标定中已知失败，本轮未改它，未宣称所有历史套件全绿。

[非落笔输出对照](pen_down_evidence_2026-10-16/non-pendown-comparison.json)：LINE/POLY/CURVE剔除落笔阶段及时间戳后，所有整数目标序列与本轮真实基线完全相同；纸面几何和正常描线推进未变。POLY和CURVE的总模拟耗时增加340ms，是新增落笔后500ms减去落笔阶段约160ms提速；不是纸面描线速度变慢。LINE则因首点1000→500且落笔提速，总时间5020→4360ms。

## 保护与回退

以下与本轮开始逐字相同：kinematics（RIGHT_ZERO=194、ELBOW_OFFSET=117）、arm、hardware_config（HOME/限位）、joystick、manual、recording、pick_place、D10驱动/协议、include/config.h及platformio.ini（POLY500/Fast环境）。path_planner的曲线增益和整个曲线公式尾部也逐字一致。

本轮开始的完整源码/配置/测试与当时固件快照：
`D:\Develop\MeArm-Bringup-2026-10-12\.pio\pendown-20261016-154741\baseline.zip`

候选HEX另存同目录candidate.hex。回退时只从ZIP选择性恢复本轮4个生产文件和相应测试；不要覆盖随后新增的实机标定。该基线N为反序；如果只想恢复功能而仍用原方向，需单独处理N顶点顺序。未执行reset、stash、清理或任何提交/推送。

建议由用户确认后自行烧录，先用已验证的纸面高度运行指定N命令，确认上述四点顺序；观察落笔更快且确认后停约0.5s，其他N顶点约1s。再对照LINE/POLY/CURVE，测试落笔移动中及500ms等待中的HOLD/GO/CANCEL。预检失败保留ERR/PATH ERR/STATUS，不扩大限位。

软件测试不证明真实笔尖已经到位、纸面压力合适、供电稳定或伺服能跟上37.5mm/s。关节限速仍可能成为其他姿态瓶颈；中断/串口耗时及40ms增量上限可能使等待墙钟时间更长。代理未操作硬件或宣称实机通过。
