#include "OLED.h"
#include "Motor.h"
#include "UI.h"

#define UI_MODE_MOTION    0 // 运行与点动模式
#define UI_MODE_FREQUENCY 1 // 频率参数设置模式

void UI_Init(void)
{
    OLED_Init();
}

void UI_Update(uint8_t ui_mode)
{
    // 显示数据由Motor模块提供，界面模块不直接修改运动状态
    OLED_Clear();
    OLED_ShowString(0, 0, ui_mode == UI_MODE_MOTION ? "MODE:RUN" :
                         (ui_mode == UI_MODE_FREQUENCY ? "MODE:FREQ" : "MODE:AMP"), OLED_6X8);
    OLED_ShowString(0, 8, Motor_IsReturnActive() ? "MOTION:RETURN" :
                         (Motor_IsMotionEnabled() ? "MOTION:ON " :
                         (Motor_HasLimitError() ? "MOTION:LIMIT" : "MOTION:OFF")), OLED_6X8);
    OLED_ShowString(0, 16, Motor_IsJogActive() ? "JOG:ON " : "JOG:OFF", OLED_6X8);
    OLED_ShowString(54, 16, Motor_IsRandomEnabled() ? "RND:ON" : "RND:OFF", OLED_6X8);
    OLED_ShowString(0, 24, "POSmm:", OLED_6X8);
    OLED_ShowSignedNum(36, 24, Motor_GetPositionMm(), 4, OLED_6X8);
    OLED_ShowString(0, 32, "FREQx100:", OLED_6X8);
    OLED_ShowNum(54, 32, Motor_GetFrequencyX100(), 3, OLED_6X8);
    OLED_ShowString(0, 40, "AMPmm:", OLED_6X8);
    OLED_ShowNum(42, 40, Motor_GetAmplitudeMm(), 3, OLED_6X8);
    OLED_Update();
}
