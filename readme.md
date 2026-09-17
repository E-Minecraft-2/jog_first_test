# STM32F103 步进电机简谐运动/点动控制

这个工程是一个基于 STM32F103C8 和 STM32F10x 标准外设库的 Keil MDK 工程。当前主程序的核心功能是通过 `TIM2_CH1` 输出步进电机脉冲，用 `PB10` 控制方向，并通过 5 个按键和 OLED 实现简谐往复运动启停、正/反方向点动、模式切换和参数显示/调整。

## 当前主程序做的事情

入口文件是 `User/main.c`，主要流程如下：

1. 初始化系统时钟，关闭 JTAG 复用，仅保留 SWD 调试。
2. 初始化 GPIO：
   - `PA0`：`TIM2_CH1` PWM 输出，作为步进驱动器的脉冲信号。
   - `PB10`：普通推挽输出，作为方向信号。
   - `PA2`：按键1，启动/停止简谐运动，低电平有效。
   - `PA3`：按键2，运行模式下负方向点动，参数模式下减小参数。
   - `PA4`：按键3，运行模式下正方向点动，参数模式下增大参数。
   - `PA5`：按键4，切换运行模式/参数模式。
   - `PA6`：按键5，当前预留。
3. 初始化 `TIM2`，在简谐运动模式下输出 100 kHz、50% 占空比 PWM。
4. 初始化 `TIM3`，每 0.5 ms 进入一次中断，累加 `sys_time_us` 作为运动计算时间基准。
5. 初始化 OLED，并周期刷新当前模式、运动状态、点动状态、位置和频率参数。
6. 主循环中：
   - 轮询 `PA2~PA6` 并做 20 ms 消抖。
   - 检测 `PA2`，在运行模式下切换简谐运动启停。
   - 在运行模式且简谐运动关闭时，检测 `PA3/PA4` 做连续点动。
   - 检测 `PA5`，切换运行模式/参数模式。
   - 在参数模式下，`PA3/PA4` 调整简谐运动频率。
   - 当时间基准更新且当前未发脉冲时，计算新的简谐目标位置并发送增量脉冲。

## 运动参数

当前参数都定义在 `User/main.c` 顶部：

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `MOTION_AMPLITUDE_MM` | `200.0f` | 简谐运动幅值，单位 mm |
| `MOTION_FREQUENCY` | `0.21f` | 简谐运动频率默认值，单位 Hz |
| `MOTION_FREQUENCY_MIN/MAX/STEP` | `0.01f / 2.00f / 0.01f` | 参数模式下频率调整范围和步进 |
| `PULSES_PER_REV` | `2000UL` | 电机/驱动器每转脉冲数 |
| `LEAD_SCREW_PITCH` | `20.0f` | 丝杆导程，单位 mm/rev |
| `PULSES_PER_MM` | `100` | 换算后的每毫米脉冲数 |
| `PWM_PULSE_FREQ` | `100000UL` | 简谐模式脉冲输出频率，单位 Hz |
| `JOG_SPEED_MMPS` | `100.0f` | 点动速度，单位 mm/s |

简谐目标位置计算公式：

```c
target_float = -AMPLITUDE_PULSES * cosf(2.0f * PI * motion_frequency * t);
```

程序每 0.5 ms 根据当前时间计算一次目标位置，把目标位置和上一次浮点位置之间的差值四舍五入成整数脉冲，再交给 `TIM2` 非阻塞发送。

## 脉冲发送逻辑

- `User_SendPulsesNonBlock()` 根据脉冲正负设置方向脚，然后启动 `TIM2`。
- `TIM2_IRQHandler()` 在每个 PWM 更新中断里累计已发送脉冲数。
- 当已发送脉冲达到目标值后，停止 `TIM2`，更新 `current_position`。
- 如果发送过程中又产生了新的增量脉冲，会累加到 `pending_delta`，当前段完成后继续发送。
- 点动模式会关闭 `TIM2` 更新中断，直接持续输出固定频率 PWM；松开按键后停止输出并恢复简谐模式的 `TIM2` 配置。

## 按键行为

| 按键 | 引脚 | 行为 |
| --- | --- | --- |
| 按键1 | `PA2` | 运行模式下，在未点动时切换简谐运动启停 |
| 按键2 | `PA3` | 运行模式下负方向点动；参数模式下频率减 `0.01 Hz` |
| 按键3 | `PA4` | 运行模式下正方向点动；参数模式下频率加 `0.01 Hz` |
| 按键4 | `PA5` | 切换运行模式/参数模式，切换时会停止当前输出 |
| 按键5 | `PA6` | 当前预留，无动作 |

如果 `PA3` 和 `PA4` 同时按下，程序会停止当前点动输出，避免两个方向同时生效。

## OLED 显示

OLED 驱动使用 `Hardware/OLED.*`，主程序会调用 `OLED_Init()` 初始化，并约每 200 ms 刷新一次。当前显示内容包括：

- `MODE`：`RUN` 或 `PARAM`
- `MOTION`：简谐运动启停状态
- `JOG`：点动状态
- `POS`：当前位置脉冲数
- `FREQx100`：频率放大 100 倍后的整数值，例如 `021` 表示 `0.21 Hz`

## 工程结构

| 目录/文件 | 作用 |
| --- | --- |
| `Project.uvprojx` | Keil MDK 工程文件，目标芯片为 `STM32F103C8` |
| `User/main.c` | 当前实际控制逻辑：步进电机简谐运动、点动、TIM2/TIM3 中断 |
| `User/stm32f10x_it.c` | 标准中断模板，当前业务中断写在 `main.c` 中 |
| `System/Delay.c` | SysTick 阻塞延时函数 |
| `Start/` | CMSIS 启动文件、系统初始化、芯片头文件 |
| `Library/` | STM32F10x 标准外设库源码 |
| `Hardware/OLED.*` | OLED 显示驱动和绘图/字符显示函数，当前 `main.c` 已调用 |
| `Hardware/Key.*` | 独立按键扫描驱动，当前未加入 Keil 工程，`main.c` 使用自己的按键读取逻辑 |
| `Hardware/LED.*` | LED 控制驱动，当前未加入 Keil 工程 |
| `Objects/`、`Listings/` | Keil 构建产物和 map/listing 文件 |

## 构建说明

使用 Keil MDK 打开 `Project.uvprojx`，选择 `Target 1` 构建即可。工程配置里：

- 目标芯片：`STM32F103C8`
- 内核：Cortex-M3
- 宏定义：`USE_STDPERIPH_DRIVER`
- Include Path：`.\Start;.\Library;.\User;.\System;.\Hardware`
- 输出目录：`Objects`
- 当前配置 `CreateHexFile` 为 `0`，默认不生成 hex 文件；如果需要烧录 hex，需要在 Keil 里打开 hex 输出选项。

现有构建日志 `Objects/Project.build_log.htm` 显示最近一次构建结果为：

```text
Program Size: Code=8682 RO-data=2678 RW-data=60 ZI-data=2676
".\Objects\Project.axf" - 0 Error(s), 0 Warning(s).
```

## 注意事项

- `Hardware/Key.c`、`Hardware/LED.c` 当前没有加入 `Project.uvprojx` 的编译列表，更多像是保留的外设驱动代码。
- `main.c` 里直接定义了 `TIM2_IRQHandler` 和 `TIM3_IRQHandler`，不要再在 `stm32f10x_it.c` 中重复定义同名中断函数。
- `User_Delay_ms()` 是简单空循环延时，精度依赖编译优化和主频；项目里另有 `System/Delay.c` 的 SysTick 延时函数。
