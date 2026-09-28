#include "AppConfig.h"
#include "Motor.h"
#include "UI.h"

#if APP_ENABLE_HMI
#include "HMI.h"

#define HMI_CMD_PAGE_SELECT 0xE0
#define HMI_CMD_STATUS      0xE1
#define HMI_PAGE_RUN        0x00
#define HMI_PAGE_HOME       0x02

static uint8_t hmi_last_return_active; // 记录上次回零状态，避免周期刷新时重复切换页面
#endif
#if APP_ENABLE_OLED
#include "OLED.h"

#define UI_MODE_MOTION    0 // 运行与点动模式
#define UI_MODE_FREQUENCY 1 // 频率参数设置模式
#endif

void UI_Init(void)
{
#if APP_ENABLE_HMI
    uint8_t page = HMI_PAGE_RUN;

    // 上电后通知串口屏进入运行界面，调整界面由屏端按钮自行切换。
    HMI_SendFrame(HMI_CMD_PAGE_SELECT, &page, 1);
#endif
#if APP_ENABLE_OLED
    OLED_Init();
#endif
}

void UI_Update(uint8_t ui_mode)
{
#if APP_ENABLE_HMI
    uint8_t data[10];
    uint8_t state;
    uint8_t return_active;
    uint8_t page;
    uint32_t position;
    uint16_t frequency;
    uint16_t amplitude;

    return_active = Motor_IsReturnActive();
    if (Motor_HasLimitError())
        state = 4;
    else if (return_active)
        state = 3;
    else if (Motor_IsMotionEnabled() && Motor_IsRandomEnabled())
        state = 2;
    else if (Motor_IsMotionEnabled())
        state = 1;
    else if (Motor_IsJogActive() || Motor_IsSending())
        state = 5;
    else
        state = 0;

    position = (uint32_t)Motor_GetPositionX100Mm();
    frequency = Motor_GetFrequencyX100();
    amplitude = Motor_GetAmplitudeX100();

    // 回零开始时进入page2，回零完成后回到page0；只在状态变化时发送切页命令。
    if (return_active != hmi_last_return_active)
    {
        page = return_active ? HMI_PAGE_HOME : HMI_PAGE_RUN;
        HMI_SendFrame(HMI_CMD_PAGE_SELECT, &page, 1);
        hmi_last_return_active = return_active;
    }

    // 运行页和调整页共用此状态帧，三个参数始终由下位机周期下发。
    // E1状态：0空闲、1运行1、2运行2、3回零、4限位错误、5点动。
    // 其余数据：位置(0.01 mm)、频率(0.01 Hz)、振幅(0.01 mm)、点动幅值(mm)。
    // 多字节字段按小端序发送，便于串口屏使用ucopy直接读取。
    data[0] = state;
    data[1] = (uint8_t)position;
    data[2] = (uint8_t)(position >> 8);
    data[3] = (uint8_t)(position >> 16);
    data[4] = (uint8_t)(position >> 24);
    data[5] = (uint8_t)frequency;
    data[6] = (uint8_t)(frequency >> 8);
    data[7] = (uint8_t)amplitude;
    data[8] = (uint8_t)(amplitude >> 8);
    data[9] = (uint8_t)Motor_GetJogAmplitudeMm();
    HMI_SendFrame(HMI_CMD_STATUS, data, sizeof(data));
#endif

#if APP_ENABLE_OLED
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
#elif !APP_ENABLE_HMI
    // OLED关闭时保留统一界面接口，后续由串口屏实现替换。
    (void)ui_mode;
#endif
}
