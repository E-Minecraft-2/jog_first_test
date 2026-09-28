#include "stm32f10x.h"
#include "App.h"
#include "AppConfig.h"
#if APP_ENABLE_HMI
#include "HMI.h"
#endif
#if APP_ENABLE_KEY
#include "Key.h"
#endif
#include "Motor.h"
#include "Timebase.h"
#include "UI.h"

#define UI_REFRESH_US 100000UL // 串口屏状态刷新周期，单位：微秒
#define UI_MODE_MOTION    0      // 运行与点动模式
#define UI_MODE_FREQUENCY 1      // 频率参数设置模式
#define UI_MODE_AMPLITUDE 2      // 幅值参数设置模式

static uint8_t ui_mode = UI_MODE_MOTION; // 当前界面和按键功能模式
static uint32_t last_time_us;            // 上次执行简谐运动计算的时间
static uint32_t last_ui_us;              // 上次刷新界面状态的时间

#if APP_ENABLE_HMI
static uint16_t App_ReadU16(const uint8_t *data)
{
    // 与淘晶驰prints/ucopy保持一致：低字节在前，高字节在后。
    return data[0] | ((uint16_t)data[1] << 8);
}

static void App_ProcessHMI(uint32_t now_us)
{
    HMI_Frame frame;
    uint8_t handled;
    uint16_t value;

    HMI_Process();
    while (HMI_GetFrame(&frame))
    {
        handled = 1;
        // 简谐运行期间只接受两个运行按钮，按下任意一个都停止运行并开始回零。
        if (Motor_IsMotionEnabled() && frame.command != 0xC1 && frame.command != 0xC2)
        {
            handled = 0;
        }
        else switch (frame.command)
        {
            case 0xA1: // 频率加/减
                if (frame.length == 0) Motor_ChangeFrequency(1); else handled = 0;
                break;
            case 0xA2: // 频率减
                if (frame.length == 0) Motor_ChangeFrequency(-1); else handled = 0;
                break;
            case 0xA3: // 幅值加
                if (frame.length == 0) Motor_ChangeAmplitude(1); else handled = 0;
                break;
            case 0xA4: // 幅值减
                if (frame.length == 0) Motor_ChangeAmplitude(-1); else handled = 0;
                break;
            case 0xA5: // 点动幅值加
                if (frame.length == 0) Motor_ChangeJogAmplitude(1); else handled = 0;
                break;
            case 0xA6: // 点动幅值减
                if (frame.length == 0) Motor_ChangeJogAmplitude(-1); else handled = 0;
                break;

            case 0xB1: // 设置频率(单位：0.01Hz)
                if (frame.length == 2)
                    Motor_SetFrequencyX100(App_ReadU16(frame.data));
                else
                    handled = 0;
                break;
            case 0xB2: // 设置幅值(单位：0.01mm)
                if (frame.length == 2)
                    Motor_SetAmplitudeX100(App_ReadU16(frame.data));
                else
                    handled = 0;
                break;
            case 0xB3: // 设置点动幅值(单位：mm)
                if (frame.length == 2)
                {
                    value = App_ReadU16(frame.data);
                    // 保留串口屏原变量名和B3命令，仅将其含义改为点动幅值(mm)。
                    Motor_SetJogAmplitudeMm(value);
                }
                else
                    handled = 0;
                break;

            case 0xC1: // 回零
                if (frame.length != 0 || Motor_IsJogActive() || Motor_IsReturnActive())
                    handled = 0;
                else if (Motor_IsMotionEnabled())
                    Motor_ReturnToStart();
                else if (Motor_IsSending())
                    handled = 0;
                else
                {
                    Motor_SetRandomEnabled(0, now_us);
                    Motor_StartMotion(now_us);
                }
                break;
            case 0xC2: // 随机运行
                if (frame.length != 0 || Motor_IsJogActive() || Motor_IsReturnActive())
                    handled = 0;
                else if (Motor_IsMotionEnabled())
                    Motor_ReturnToStart();
                else if (Motor_IsSending())
                    handled = 0;
                else
                {
                    Motor_SetRandomEnabled(1, now_us);
                    if (Motor_IsRandomEnabled())
                        Motor_StartMotion(now_us);
                }
                break;
            case 0xC3: // 点动正向
                if (frame.length == 0) Motor_JogStep(1); else handled = 0;
                break;
            case 0xC4: // 点动反向
                if (frame.length == 0) Motor_JogStep(-1); else handled = 0;
                break;
            case 0xD1: // 停止运动
                if (frame.length == 0) Motor_ReturnToStart(); else handled = 0;
                break;
            default:
                handled = 0;
                break;
        }

        // 状态帧同时作为有效命令的执行反馈。
        if (handled)
            UI_Update(ui_mode);
    }
}
#endif

#if APP_ENABLE_KEY
static void App_ProcessParameters(void)
{
    // 参数模式下，负/正点动键分别作为减/加按键使用
    uint8_t changed = 0;

    if (ui_mode == UI_MODE_FREQUENCY && Key_GetEvent(KEY_NEG))
    {
        Motor_ChangeFrequency(-1);
        changed = 1;
    }
    if (ui_mode == UI_MODE_FREQUENCY && Key_GetEvent(KEY_POS))
    {
        Motor_ChangeFrequency(1);
        changed = 1;
    }
    if (ui_mode == UI_MODE_AMPLITUDE && Key_GetEvent(KEY_NEG))
    {
        Motor_ChangeAmplitude(-1);
        changed = 1;
    }
    if (ui_mode == UI_MODE_AMPLITUDE && Key_GetEvent(KEY_POS))
    {
        Motor_ChangeAmplitude(1);
        changed = 1;
    }
    if (changed)
        UI_Update(ui_mode);
}
#endif

void App_Init(void)
{
#if APP_ENABLE_KEY
    Key_Init();
#endif
    Motor_Init();
    Timebase_Init();
#if APP_ENABLE_HMI
    HMI_Init();
#endif
    UI_Init();
    UI_Update(ui_mode);
}

/** @brief 处理应用程序逻辑
 * 
 * @param now_us 
 */
void App_Process(void)
{
    // 主循环按固定顺序完成按键、模式、运动和界面处理
    uint32_t now = Timebase_GetUs();

#if APP_ENABLE_HMI
    App_ProcessHMI(now);
#endif
#if APP_ENABLE_KEY
    Key_Scan(now);
#endif
    if (Motor_IsReturnActive() && !Motor_IsSending())
    {
        Motor_Stop();
        UI_Update(ui_mode);
    }
#if APP_ENABLE_KEY
    if (Key_GetEvent(KEY_MODE))
    {
        Motor_Stop();
        ui_mode++;
        if (ui_mode > UI_MODE_AMPLITUDE)
            ui_mode = UI_MODE_MOTION;
        Key_ClearEvents();
        UI_Update(ui_mode);
    }
    if (ui_mode == UI_MODE_MOTION && Key_GetEvent(KEY_START) &&
        !Motor_IsJogActive() && !Motor_IsReturnActive())
    {
        if (Motor_IsMotionEnabled())
            Motor_ReturnToStart();
        else
            Motor_StartMotion(now);
        UI_Update(ui_mode);
    }
    if (ui_mode == UI_MODE_MOTION && Key_GetEvent(KEY_RANDOM) && !Motor_IsJogActive())
    {
        Motor_ToggleRandom(now);
        UI_Update(ui_mode);
    }
    if (ui_mode == UI_MODE_FREQUENCY || ui_mode == UI_MODE_AMPLITUDE)
        App_ProcessParameters();
    else
        Motor_ProcessJog(Key_IsPressed(KEY_NEG), Key_IsPressed(KEY_POS));
#endif

    if (now != last_time_us)
    {
        last_time_us = now;
        if (ui_mode == UI_MODE_MOTION && Motor_IsMotionEnabled() &&
            !Motor_IsJogActive() && !Motor_IsSending())
            Motor_Update(now);
    }
    if ((uint32_t)(now - last_ui_us) >= UI_REFRESH_US)
    {
        last_ui_us = now;
        UI_Update(ui_mode);
    }
}
