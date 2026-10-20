# 通用 ST 简谐轨迹功能块

`FB_Harmonic.st` 提取当前 STM32 程序中普通简谐运动的轨迹计算，输出单位为 mm 的绝对位置。它不包含随机幅值扰动、机械回零、点动、返回起点或厂商轴指令。

## 轨迹和原程序的对应关系

```text
x(t) = S + A × [1 − cos(2πft)]
S：启动时的轴位置，mm
A：幅值，mm；往复总行程为 2A
f：频率，Hz；运动周期 T = 1/f
```

这是原程序从行程下端点出发的正弦类位置轨迹，起始速度为零，启动后连续往复。

| 时间 | 目标位置 |
| --- | --- |
| 0 | S |
| T/4 | S + A |
| T/2 | S + 2A |
| 3T/4 | S + A |
| T | S |

默认 A=200 mm、f=0.21 Hz、S=0 mm，对应 0～400 mm 往复，周期约 4.762 s。

## 接口

| 输入 | 默认值 | 含义 |
| --- | --- | --- |
| Enable | FALSE | 上升沿启动，保持 TRUE 连续运行，FALSE 停止更新 |
| StartPositionMm | 0.0 | 启动时轴的当前位置，坐标必须已建立 |
| AmplitudeMm | 200.0 | 幅值，允许 10～470 mm |
| FrequencyHz | 0.21 | 频率，允许 0.01～2 Hz |
| CycleTimeSeconds | 0.0005 | 功能块调用间隔，单位秒，必须等于固定任务周期 |

| 输出 | 含义 |
| --- | --- |
| Active | 正在生成轨迹；仅在 TRUE 时把 PositionMm 作为有效轨迹使用 |
| Error | 启动校验失败，保持到 Enable=FALSE |
| ErrorID | 0：无错误；1：幅值非法；2：频率非法；3：周期非法；4：行程越界 |
| PositionMm | 绝对目标位置，mm；停用或启动失败时保持上次输出 |

启动时检查 `S ≥ 0` 且 `S + 2A ≤ 950 mm`。参数在启动时锁存，运行中修改输入不会改变当前轨迹。重新启动前，必须至少调用一次 Enable=FALSE；再次启动会从新传入的起点重新计时，不续接旧相位。

周期必须大于零，并满足 `f × CycleTimeSeconds < 1`；这只是基本合法性检查。实际应根据轴接口和轨迹精度使用足够小的调用周期。

## 调用示例

创建 `FB_Harmonic` 类型的实例，在固定周期任务中每周期调用一次，包括停用期间。下例假设任务周期为 1 ms；如果配置为 0.5 ms，把 CycleTimeSeconds 改成 0.0005。

```iecst
PROGRAM PRG_HarmonicExample
VAR
    Harmonic         : FB_Harmonic;
    RunRequest       : BOOL := FALSE;
    AxisPositionMm   : LREAL := LREAL#0.0;
    AxisTargetMm     : LREAL := LREAL#0.0;
    TrajectoryActive : BOOL;
    TrajectoryError  : BOOL;
    TrajectoryErrorID : UINT;
END_VAR

(* AxisPositionMm must be connected to the axis position in this coordinate system. *)
Harmonic(
    Enable := RunRequest,
    StartPositionMm := AxisPositionMm,
    AmplitudeMm := LREAL#200.0,
    FrequencyHz := LREAL#0.21,
    CycleTimeSeconds := LREAL#0.001
);

TrajectoryActive := Harmonic.Active;
TrajectoryError := Harmonic.Error;
TrajectoryErrorID := Harmonic.ErrorID;
IF Harmonic.Active THEN
    AxisTargetMm := Harmonic.PositionMm;
END_IF;
END_PROGRAM
```

AxisPositionMm、RunRequest 和 AxisTargetMm 是示例变量，需接入实际轴程序。上升沿所在扫描输出 S，之后每次调用相位前进 `2πf × CycleTimeSeconds`。相位每圈回绕，避免长时间累计一个不断增大的时间值。

## 对接伺服

连续位置给定接口可使用 PositionMm；具体接入方式由 PLC 运动库确定。若采用脉冲＋方向，原工程比例为 100 pulse/mm，但这个功能块不产生高速脉冲，也不管理脉冲批次的 Busy/Done 握手。普通定位指令是否支持每周期更新目标，需要按型号确认。

原 STM32 以 0.5 ms 时基计算，并在上一批脉冲忙时跳过计算；本块按固定任务周期推进相位，每次调用都更新目标，因此保留数学轨迹，而不复制 100 kHz 分批脉冲时序。任务丢周期会使本块轨迹时间变慢；必须保证调用频率，不要放在时间不固定的普通扫描中并假设它恒为 0.5 ms。

Enable=FALSE 仅停止轨迹更新，不能替代轴停止指令。外部轴程序应处理限位、报警、停机和返回起点；重新启动前应确保轴已停止并用当前位置作为 S。

评估轴能力时，曲线最大速度为 `2πfA`，最大加速度为 `(2πf)²A`。默认参数约为 263.9 mm/s 和 348.2 mm/s²；本块只校验参数和行程，不校验具体轴的速度、加速度能力。

## 导入与验证范围

这是通用 IEC ST 源码，未绑定厂商工程。若编辑器把声明和实现分开，将 FUNCTION_BLOCK 与各 VAR 区域放入声明区，将 IF 开始至 PreviousEnable 赋值的逻辑放入实现区，按编辑器要求处理外层结束标记。

COS 的输入相位使用弧度，采用 LREAL 计算，参考 [CODESYS 官方 COS 文档](https://content.helpme-codesys.com/en/CODESYS%20Development%20System/_cds_operator_cos.html)。是否支持这些类型和源文件导入方式，以目标 PLC 编译器为准。

本地验证将本文件源码使用的 ST 语句子集转换为 Python 执行，检查轨迹关键点、相位回绕、启停、参数锁存和行程边界，共通过 30066 项断言；默认轨迹连续 30000 个采样点与公式的最大偏差约为 6.4×10⁻¹¹ mm。此检查不能代替 PLC 编译和运行。当前环境没有 PLC ST 编译器，尚未在目标 PLC 上编译或运行。
