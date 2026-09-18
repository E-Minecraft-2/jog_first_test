#include "stm32f10x.h"
#include "App.h"
#include "Key.h"
#include "Motor.h"
#include "Timebase.h"
#include "UI.h"

#define OLED_REFRESH_US 200000UL // OLED定时刷新周期，单位：微秒
#define UI_MODE_MOTION    0      // 运行与点动模式
#define UI_MODE_FREQUENCY 1      // 频率参数设置模式
#define UI_MODE_AMPLITUDE 2      // 幅值参数设置模式

static uint8_t ui_mode = UI_MODE_MOTION; // 当前界面和按键功能模式
static uint32_t last_time_us;            // 上次执行简谐运动计算的时间
static uint32_t last_oled_us;            // 上次刷新OLED的时间

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

void App_Init(void)
{
    Key_Init();
    Motor_Init();
    Timebase_Init();
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

    Key_Scan(now);
    if (Motor_IsReturnActive() && !Motor_IsSending())
    {
        Motor_Stop();
        UI_Update(ui_mode);
    }
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

    if (now != last_time_us)
    {
        last_time_us = now;
        if (ui_mode == UI_MODE_MOTION && Motor_IsMotionEnabled() &&
            !Motor_IsJogActive() && !Motor_IsSending())
            Motor_Update(now);
    }
    if ((uint32_t)(now - last_oled_us) >= OLED_REFRESH_US)
    {
        last_oled_us = now;
        UI_Update(ui_mode);
    }
}
