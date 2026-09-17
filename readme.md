# STM32F103 步进电机简谐运动控制

这是一个基于 `STM32F103C8` 和 STM32F10x 标准外设库的 Keil MDK 工程。当前主程序通过 `TIM2_CH1` 输出步进电机脉冲，使用 `PB10` 控制方向，并通过 5 个按键和 OLED 完成简谐运动启停、正反方向点动、随机扰动开关、频率/幅值调节和状态显示。

入口文件是 `User/main.c`。

## 当前功能

- `PA0` 输出 `TIM2_CH1` PWM，作为步进驱动器脉冲信号。
- `PB10` 输出方向信号。
- `TIM3` 每 0.5 ms 进入一次中断，累加 `sys_time_us` 作为运动计算时间基准。
- 简谐运动模式下，程序按当前频率和幅值计算目标位置，并把位置增量转换为脉冲交给 `TIM2` 非阻塞发送。
- 点动模式下，按键按住后持续输出固定频率 PWM，松开后停止输出。
- 支持 0 到 950 mm 的行程保护，启动简谐运动或随机扰动前会检查当前参数是否越界。
- OLED 约每 200 ms 刷新一次，显示模式、运动状态、点动状态、随机扰动状态、当前位置、频率和幅值。

## 主要参数

这些参数定义在 `User/main.c` 顶部。

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `MOTION_AMPLITUDE_MM` | `200.0f` | 简谐运动默认幅值，单位 mm |
| `MOTION_AMPLITUDE_MIN/MAX/STEP` | `10.0f / 470.0f / 10.0f` | 幅值调节范围和步长，单位 mm |
| `MOTION_FREQUENCY` | `0.21f` | 简谐运动默认频率，单位 Hz |
| `MOTION_FREQUENCY_MIN/MAX/STEP` | `0.01f / 2.00f / 0.01f` | 频率调节范围和步长，单位 Hz |
| `RANDOM_DISTURBANCE_MM` | `10.0f` | 随机扰动幅值，单位 mm |
| `RANDOM_UPDATE_US` | `100000UL` | 随机扰动更新周期，单位 us |
| `JOG_SPEED_MMPS` | `100.0f` | 点动速度，单位 mm/s |
| `PULSES_PER_REV` | `2000UL` | 电机/驱动器每转脉冲数 |
| `LEAD_SCREW_PITCH` | `20.0f` | 丝杆导程，单位 mm/rev |
| `PULSES_PER_MM` | `100` | 换算后的每毫米脉冲数 |
| `MAX_TRAVEL_MM` | `950.0f` | 电缸最大运行长度，单位 mm |
| `PWM_PULSE_FREQ` | `100000UL` | 简谐运动脉冲输出频率，单位 Hz |

简谐运动目标位置计算公式：

```c
target_float = motion_start_position +
               (motion_amplitude * PULSES_PER_MM + random_disturbance_pulses) *
               (1.0f - cosf(2.0f * PI * motion_frequency * t));
```

程序会把目标位置和上一次浮点位置之间的差值四舍五入成整数脉冲，再交给 `User_SendPulsesNonBlock()` 输出。

## 按键行为

| 按键 | 引脚 | 行为 |
| --- | --- | --- |
| 按键 1 | `PA2` | 在运行模式下启动/停止简谐运动 |
| 按键 2 | `PA3` | 运行模式下负方向点动；频率/幅值模式下减小参数 |
| 按键 3 | `PA4` | 运行模式下正方向点动；频率/幅值模式下增大参数 |
| 按键 4 | `PA5` | 在运行模式下打开/关闭随机扰动 |
| 按键 5 | `PA6` | 切换 `RUN`、`FREQ`、`AMP` 三种界面模式 |

`PA3` 和 `PA4` 同时按下时，程序会停止当前点动输出，避免两个方向同时生效。

## OLED 显示

OLED 驱动位于 `Hardware/OLED.*`，主程序调用 `OLED_Init()` 初始化，并调用 `User_UpdateOLED()` 刷新显示。当前显示内容包括：

- `MODE:RUN/FREQ/AMP`：当前界面模式
- `MOTION:ON/OFF/LIMIT`：简谐运动状态或行程限制提示
- `JOG:ON/OFF`：点动状态
- `RND:ON/OFF`：随机扰动状态
- `POSmm`：当前位置，单位 mm
- `FREQx100`：频率放大 100 倍后的整数值，例如 `021` 表示 `0.21 Hz`
- `AMPmm`：当前简谐运动幅值，单位 mm

## 工程结构

| 目录/文件 | 作用 |
| --- | --- |
| `Project.uvprojx` | Keil MDK 工程文件，目标芯片为 `STM32F103C8` |
| `User/main.c` | 当前控制逻辑：按键、OLED、简谐运动、点动、随机扰动、TIM2/TIM3 中断 |
| `User/stm32f10x_it.c` | 标准中断模板；当前业务中断写在 `main.c` 中 |
| `Hardware/OLED.*` | OLED 显示驱动 |
| `Hardware/Key.*` | 独立按键扫描驱动，当前主程序未使用 |
| `Hardware/LED.*` | LED 控制驱动，当前主程序未使用 |
| `System/Delay.c` | SysTick 延时函数 |
| `Start/` | CMSIS 启动文件、系统初始化和芯片头文件 |
| `Library/` | STM32F10x 标准外设库源码 |
| `Objects/`、`Listings/` | Keil 构建产物和 listing/map 文件 |

## 构建说明

使用 Keil MDK 打开 `Project.uvprojx`，选择 `Target 1` 构建即可。当前工程配置要点：

- 目标芯片：`STM32F103C8`
- 内核：Cortex-M3
- 宏定义：`USE_STDPERIPH_DRIVER`
- Include Path：`.\Start;.\Library;.\User;.\System;.\Hardware`
- 输出目录：`Objects`
- 当前 `CreateHexFile` 为 `0`，默认不生成 hex 文件；如果需要烧录 hex，需要在 Keil 中打开 hex 输出选项。

## 注意事项

- `main.c` 中已经定义了 `TIM2_IRQHandler` 和 `TIM3_IRQHandler`，不要在 `stm32f10x_it.c` 中重复定义同名中断函数。
- `Hardware/Key.c` 和 `Hardware/LED.c` 当前没有加入 `Project.uvprojx` 的编译列表，更像是保留的外设驱动代码。
- `User_Delay_ms()` 是简单空循环延时，精度依赖编译优化和主频；项目里另有 `System/Delay.c` 的 SysTick 延时函数。
