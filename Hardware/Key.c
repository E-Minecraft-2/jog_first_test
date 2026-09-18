#include "stm32f10x.h"
#include "Key.h"

#define KEY_DEBOUNCE_US 20000UL // 按键消抖时间，单位：微秒

// key_pressed保存稳定按下状态，key_event只在确认按下时置位一次
static uint8_t key_pressed[KEY_COUNT];
static uint8_t key_event[KEY_COUNT];

void Key_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
}

void Key_Scan(uint32_t now_us)
{
    // 顺序与Key_Id保持一致：启动、负点动、正点动、随机扰动、模式切换
    static const uint16_t key_pins[KEY_COUNT] = {
        GPIO_Pin_2, GPIO_Pin_6, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_3
    };
    static uint8_t raw_last[KEY_COUNT] = {1, 1, 1, 1, 1};
    static uint8_t stable[KEY_COUNT] = {1, 1, 1, 1, 1};
    static uint32_t change_time[KEY_COUNT];
    uint8_t i;
    uint8_t raw;

    for (i = 0; i < KEY_COUNT; i++)
    {
        raw = GPIO_ReadInputDataBit(GPIOA, key_pins[i]);
        if (raw != raw_last[i])
        {
            raw_last[i] = raw;
            change_time[i] = now_us;
        }
        if (raw != stable[i] && (uint32_t)(now_us - change_time[i]) >= KEY_DEBOUNCE_US)
        {
            stable[i] = raw;
            key_pressed[i] = (stable[i] == 0);
            if (key_pressed[i])
                key_event[i] = 1;
        }
    }
}

void Key_ClearEvents(void)
{
    uint8_t i;
    for (i = 0; i < KEY_COUNT; i++)
        key_event[i] = 0;
}

uint8_t Key_GetEvent(Key_Id key)
{
    // 读取事件后立即清除，避免同一次按下被重复处理
    uint8_t event;
    if (key >= KEY_COUNT)
        return 0;
    event = key_event[key];
    key_event[key] = 0;
    return event;
}

uint8_t Key_IsPressed(Key_Id key)
{
    if (key >= KEY_COUNT)
        return 0;
    return key_pressed[key];
}
