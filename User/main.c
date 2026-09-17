#include "stm32f10x.h"
#include "OLED.h"
#include <stdint.h>
#include <math.h>

// ==================== 用户参数设置 ====================
#define MOTION_AMPLITUDE_MM   200.0f // 简谐运动默认幅值，单位：mm
#define MOTION_AMPLITUDE_MIN  10.0f  // 简谐运动幅值最小值，单位：mm
#define MOTION_AMPLITUDE_MAX  470.0f // 简谐运动幅值最大值，单位：mm（10mm步进）
#define MOTION_AMPLITUDE_STEP 10.0f  // 简谐运动幅值调节步长，单位：mm
#define MOTION_FREQUENCY      0.21f  // 简谐运动默认频率，单位：Hz
#define MOTION_FREQUENCY_MIN  0.01f  // 简谐运动频率最小值，单位：Hz
#define MOTION_FREQUENCY_MAX  2.00f  // 简谐运动频率最大值，单位：Hz
#define MOTION_FREQUENCY_STEP 0.01f  // 简谐运动频率调节步长，单位：Hz
#define RANDOM_DISTURBANCE_MM 10.0f  // 随机扰动幅值，单位：mm
#define RANDOM_UPDATE_US      100000UL // 随机扰动更新周期，单位：微秒

// ==================== 点动参数 ====================
#define JOG_SPEED_MMPS        100.0f // 点动速度，单位：mm/s

// ==================== 固定参数（齿轮比 5:1） ====================
#define PULSES_PER_REV        2000UL // 每转脉冲数（含细分）
#define LEAD_SCREW_PITCH      20.0f  // 滚珠丝杆导程，单位：mm
#define PULSES_PER_MM         (PULSES_PER_REV / LEAD_SCREW_PITCH)              // 每毫米脉冲数
#define MAX_TRAVEL_MM         950.0f // 电缸最大运行长度，单位：mm
#define MAX_TRAVEL_PULSES     ((int32_t)(MAX_TRAVEL_MM * PULSES_PER_MM))        // 电缸最大运行脉冲数
#define RANDOM_DISTURBANCE_PULSES ((int32_t)(RANDOM_DISTURBANCE_MM * PULSES_PER_MM)) // 随机扰动幅值，单位：脉冲

// TIM2 PWM载波频率，简谐运动模式使用
#define PWM_PULSE_FREQ        100000UL                        // PWM脉冲频率，单位：Hz
#define ARR_VALUE_HARMONIC    (1000000 / PWM_PULSE_FREQ - 1) // TIM2自动重装载值
#define HALF_PULSE_HARMONIC   ((ARR_VALUE_HARMONIC + 1) / 2) // TIM2比较匹配值，产生50%占空比

#define PI                    3.1415926f

// ==================== 硬件引脚 ====================
#define PULSE_PIN             GPIO_Pin_0   // PA0 (TIM2_CH1)
#define PULSE_PORT            GPIOA
#define DIR_PIN               GPIO_Pin_10  // PB10 方向
#define DIR_PORT              GPIOB

#define KEY_START_PIN         GPIO_Pin_2   // PA2 按键1：启动/停止
#define KEY_JOG_NEG_PIN       GPIO_Pin_3   // PA3 按键2：负方向点动/参数-
#define KEY_JOG_POS_PIN       GPIO_Pin_4   // PA4 按键3：正方向点动/参数+
#define KEY_MODE_PIN          GPIO_Pin_5   // PA5 按键4：随机扰动开关
#define KEY_FUNC_PIN          GPIO_Pin_6   // PA6 按键5：切换模式
#define KEY_PORT              GPIOA

#define KEY_COUNT             5
#define KEY_DEBOUNCE_US       20000UL
#define OLED_REFRESH_US       200000UL

#define UI_MODE_MOTION        0
#define UI_MODE_FREQUENCY     1
#define UI_MODE_AMPLITUDE     2

#define KEY_START             0
#define KEY_NEG               1
#define KEY_POS               2
#define KEY_RANDOM            3
#define KEY_MODE              4

// ==================== 全局变量 ====================
volatile uint32_t sys_time_us = 0;           // 系统时间，单位：微秒
volatile uint32_t last_time_us = 0;          // 上次处理时间，单位：微秒
volatile int32_t current_position = 0;       // 当前脉冲位置，单位：脉冲
static float pos_float = 0.0f; // 当前脉冲位置，单位：脉冲（浮点数，用于简谐运动计算）
volatile int32_t target_pulses = 0;          // 目标脉冲数，单位：脉冲
volatile uint32_t sent_pulses = 0;          // 已发送脉冲数，单位：脉冲 
volatile uint8_t sending = 0;
volatile int32_t pending_delta = 0;
volatile uint8_t pending_flag = 0;
volatile uint8_t motion_enable = 0;
volatile uint8_t jog_active = 0;
volatile uint8_t ui_mode = UI_MODE_MOTION;

static float motion_frequency = MOTION_FREQUENCY;
static float motion_amplitude = MOTION_AMPLITUDE_MM;
static int32_t motion_start_position = 0;
static uint32_t motion_start_time_us = 0;
static uint8_t motion_limit_error = 0;
static uint8_t random_disturbance_enable = 0;
static int32_t random_disturbance_pulses = 0;
static uint32_t random_update_time_us = 0;
static uint32_t random_state = 0x13579BDFUL;
static uint8_t key_pressed[KEY_COUNT] = {0};
static uint8_t key_event[KEY_COUNT] = {0};

// ==================== 函数声明 ====================
void User_GPIO_Init(void);                       // GPIO初始化
void User_TIM2_Init_Harmonic(void);              // TIM2简谐运动模式初始化
void User_TIM2_Init_Jog(void);         
void User_TIM3_Base_Init(void);
void User_SendPulsesNonBlock(int32_t pulses);    // 非阻塞发送脉冲
void User_Motion_Calc(void);                     // 简谐运动计算
void User_Delay_ms(uint32_t ms);          
void User_Jog_Process(void); 
void User_Key_Scan(void);
void User_Key_ClearEvents(void);
uint8_t User_Key_GetEvent(uint8_t key);
uint8_t User_Key_IsPressed(uint8_t key);
void User_StopMotion(void);
void User_Param_Process(void);
void User_UpdateOLED(void);
uint8_t User_MotionFitsTravel(int32_t start_position, uint8_t random_enable);
int32_t User_RandomDisturbancePulses(void);

// ==================== 主函数 ====================
int main(void)
{
    uint32_t last_oled_us = 0;

    SystemInit();
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    User_GPIO_Init();
    User_TIM2_Init_Harmonic();
    User_TIM3_Base_Init();
    OLED_Init();

    TIM_Cmd(TIM2, DISABLE);
    GPIO_ResetBits(PULSE_PORT, PULSE_PIN);
    GPIO_SetBits(DIR_PORT, DIR_PIN);
    User_UpdateOLED();

    while (1)
    {
        User_Key_Scan();

        if (User_Key_GetEvent(KEY_MODE))
        {
            User_StopMotion();
            ui_mode++;
            if (ui_mode > UI_MODE_AMPLITUDE)
                ui_mode = UI_MODE_MOTION;
            User_Key_ClearEvents();
            User_UpdateOLED();
        }

        if (ui_mode == UI_MODE_MOTION && User_Key_GetEvent(KEY_START) && !jog_active)
        {
            if (motion_enable)
            {
                User_StopMotion();
            }
            else
            {
                if (User_MotionFitsTravel(current_position, random_disturbance_enable))
                {
                    motion_enable = 1;
                    motion_limit_error = 0;
                    motion_start_position = current_position;
                    motion_start_time_us = sys_time_us;
                    random_disturbance_pulses = 0;
                    random_update_time_us = sys_time_us;
                    pos_float = (float)current_position;
                    User_TIM2_Init_Harmonic();
                }
                else
                {
                    motion_limit_error = 1;
                }
            }
            User_UpdateOLED();
        }

        if (ui_mode == UI_MODE_MOTION && User_Key_GetEvent(KEY_RANDOM) && !jog_active)
        {
            if (random_disturbance_enable)
            {
                random_disturbance_enable = 0;
                random_disturbance_pulses = 0;
            }
            else if (User_MotionFitsTravel(motion_enable ? motion_start_position : current_position, 1))
            {
                random_disturbance_enable = 1;
                motion_limit_error = 0;
                random_disturbance_pulses = 0;
                random_update_time_us = sys_time_us;
            }
            else
            {
                motion_limit_error = 1;
            }
            User_UpdateOLED();
        }

        if (ui_mode == UI_MODE_FREQUENCY || ui_mode == UI_MODE_AMPLITUDE)
            User_Param_Process();
        else
            User_Jog_Process();

        if (sys_time_us != last_time_us)
        {
            last_time_us = sys_time_us;
            if (ui_mode == UI_MODE_MOTION && motion_enable && !jog_active && !sending)
                User_Motion_Calc();
        }

        if ((uint32_t)(sys_time_us - last_oled_us) >= OLED_REFRESH_US)
        {
            last_oled_us = sys_time_us;
            User_UpdateOLED();
        }
    }
}

// ==================== 按键轮询 ====================
void User_Key_Scan(void)
{
    static const uint16_t key_pins[KEY_COUNT] = {
        KEY_START_PIN, KEY_JOG_NEG_PIN, KEY_JOG_POS_PIN, KEY_MODE_PIN, KEY_FUNC_PIN
    };
    static uint8_t raw_last[KEY_COUNT] = {1, 1, 1, 1, 1};
    static uint8_t stable[KEY_COUNT] = {1, 1, 1, 1, 1};
    static uint32_t change_time[KEY_COUNT] = {0};
    uint8_t i;
    uint8_t raw;

    for (i = 0; i < KEY_COUNT; i++)
    {
        raw = GPIO_ReadInputDataBit(KEY_PORT, key_pins[i]);
        if (raw != raw_last[i])
        {
            raw_last[i] = raw;
            change_time[i] = sys_time_us;
        }

        if (raw != stable[i] && (uint32_t)(sys_time_us - change_time[i]) >= KEY_DEBOUNCE_US)
        {
            stable[i] = raw;
            key_pressed[i] = (stable[i] == 0);
            if (key_pressed[i])
                key_event[i] = 1;
        }
    }
}

void User_Key_ClearEvents(void)
{
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++)
        key_event[i] = 0;
}

uint8_t User_Key_GetEvent(uint8_t key)
{
    uint8_t event;

    if (key >= KEY_COUNT)
        return 0;

    event = key_event[key];
    key_event[key] = 0;
    return event;
}

uint8_t User_Key_IsPressed(uint8_t key)
{
    if (key >= KEY_COUNT)
        return 0;

    return key_pressed[key];
}

// ==================== 停止输出 ====================
void User_StopMotion(void)
{
    motion_enable = 0;
    jog_active = 0;
    sending = 0;
    sent_pulses = 0;
    pending_delta = 0;
    pending_flag = 0;

    TIM_Cmd(TIM2, DISABLE);
    TIM2->CCER &= ~TIM_CCER_CC1E;
    GPIO_ResetBits(PULSE_PORT, PULSE_PIN);
    User_TIM2_Init_Harmonic();
}

// ==================== 参数模式 ====================
void User_Param_Process(void)
{
    uint8_t changed = 0;

    if (ui_mode == UI_MODE_FREQUENCY && User_Key_GetEvent(KEY_NEG))
    {
        if (motion_frequency > MOTION_FREQUENCY_MIN + MOTION_FREQUENCY_STEP)
            motion_frequency -= MOTION_FREQUENCY_STEP;
        else
            motion_frequency = MOTION_FREQUENCY_MIN;
        changed = 1;
    }

    if (ui_mode == UI_MODE_FREQUENCY && User_Key_GetEvent(KEY_POS))
    {
        if (motion_frequency < MOTION_FREQUENCY_MAX - MOTION_FREQUENCY_STEP)
            motion_frequency += MOTION_FREQUENCY_STEP;
        else
            motion_frequency = MOTION_FREQUENCY_MAX;
        changed = 1;
    }

    if (ui_mode == UI_MODE_AMPLITUDE && User_Key_GetEvent(KEY_NEG))
    {
        if (motion_amplitude > MOTION_AMPLITUDE_MIN + MOTION_AMPLITUDE_STEP)
            motion_amplitude -= MOTION_AMPLITUDE_STEP;
        else
            motion_amplitude = MOTION_AMPLITUDE_MIN;
        changed = 1;
    }

    if (ui_mode == UI_MODE_AMPLITUDE && User_Key_GetEvent(KEY_POS))
    {
        if (motion_amplitude < MOTION_AMPLITUDE_MAX - MOTION_AMPLITUDE_STEP)
            motion_amplitude += MOTION_AMPLITUDE_STEP;
        else
            motion_amplitude = MOTION_AMPLITUDE_MAX;
        changed = 1;
    }

    if (changed)
        User_UpdateOLED();
}

// ==================== OLED显示 ====================
void User_UpdateOLED(void)
{
    uint16_t freq_x100 = (uint16_t)(motion_frequency * 100.0f + 0.5f);
    uint16_t amplitude_mm = (uint16_t)(motion_amplitude + 0.5f);
    int32_t position_mm = current_position / PULSES_PER_MM;

    OLED_Clear();
    OLED_ShowString(0, 0, ui_mode == UI_MODE_MOTION ? "MODE:RUN" :
                         (ui_mode == UI_MODE_FREQUENCY ? "MODE:FREQ" : "MODE:AMP"), OLED_6X8);
    OLED_ShowString(0, 8, motion_enable ? "MOTION:ON " :
                         (motion_limit_error ? "MOTION:LIMIT" : "MOTION:OFF"), OLED_6X8);
    OLED_ShowString(0, 16, jog_active ? "JOG:ON " : "JOG:OFF", OLED_6X8);
    OLED_ShowString(54, 16, random_disturbance_enable ? "RND:ON" : "RND:OFF", OLED_6X8);
    OLED_ShowString(0, 24, "POSmm:", OLED_6X8);
    OLED_ShowSignedNum(36, 24, position_mm, 4, OLED_6X8);
    OLED_ShowString(0, 32, "FREQx100:", OLED_6X8);
    OLED_ShowNum(54, 32, freq_x100, 3, OLED_6X8);
    OLED_ShowString(0, 40, "AMPmm:", OLED_6X8);
    OLED_ShowNum(42, 40, amplitude_mm, 3, OLED_6X8);
    OLED_Update();
}

uint8_t User_MotionFitsTravel(int32_t start_position, uint8_t random_enable)
{
    int32_t amplitude_pulses;

    amplitude_pulses = (int32_t)(motion_amplitude * PULSES_PER_MM);
    if (random_enable)
        amplitude_pulses += RANDOM_DISTURBANCE_PULSES;

    return start_position >= 0 &&
           start_position <= MAX_TRAVEL_PULSES - 2 * amplitude_pulses;
}

int32_t User_RandomDisturbancePulses(void)
{
    random_state = random_state * 1664525UL + 1013904223UL;
    return (int32_t)(random_state % (2 * RANDOM_DISTURBANCE_PULSES + 1)) -
           RANDOM_DISTURBANCE_PULSES;
}

// ==================== 点动处理 ====================
void User_Jog_Process(void)
{
    uint8_t neg_pressed;
    uint8_t pos_pressed;

    if (motion_enable || ui_mode != UI_MODE_MOTION)
        return;

    neg_pressed = User_Key_IsPressed(KEY_NEG);
    pos_pressed = User_Key_IsPressed(KEY_POS);

    if (neg_pressed && pos_pressed)
    {
        if (jog_active)
        {
            jog_active = 0;
            TIM_Cmd(TIM2, DISABLE);
            GPIO_ResetBits(PULSE_PORT, PULSE_PIN);
            User_TIM2_Init_Harmonic();
        }
        return;
    }

    if (neg_pressed || pos_pressed)
    {
        if (!jog_active)
        {
            jog_active = 1;
            TIM_Cmd(TIM2, DISABLE);
            User_TIM2_Init_Jog();
            pending_delta = 0;
            pending_flag = 0;
            sending = 0;
            sent_pulses = 0;

            if (neg_pressed)
                GPIO_ResetBits(DIR_PORT, DIR_PIN);
            else
                GPIO_SetBits(DIR_PORT, DIR_PIN);

            if ((neg_pressed && current_position > 0) ||
                (pos_pressed && current_position < MAX_TRAVEL_PULSES))
            {
                TIM2->CCER |= TIM_CCER_CC1E;
                TIM_Cmd(TIM2, ENABLE);
            }
            else
            {
                jog_active = 0;
            }
        }
        else
        {
            if (neg_pressed)
                GPIO_ResetBits(DIR_PORT, DIR_PIN);
            else
                GPIO_SetBits(DIR_PORT, DIR_PIN);
        }
    }
    else
    {
        if (jog_active)
        {
            jog_active = 0;
            TIM_Cmd(TIM2, DISABLE);
            TIM2->CCER &= ~TIM_CCER_CC1E;
            GPIO_ResetBits(PULSE_PORT, PULSE_PIN);
            User_TIM2_Init_Harmonic();
        }
    }
}

// ==================== GPIO初始化 ====================
void User_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = DIR_PIN;
    GPIO_Init(DIR_PORT, &gpio);

    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = KEY_START_PIN | KEY_JOG_NEG_PIN | KEY_JOG_POS_PIN | KEY_MODE_PIN | KEY_FUNC_PIN;
    GPIO_Init(KEY_PORT, &gpio);
}

// ==================== TIM2简谐运动模式 ====================
void User_TIM2_Init_Harmonic(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &gpio);

    tim.TIM_Period = ARR_VALUE_HARMONIC;
    tim.TIM_Prescaler = 71;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = HALF_PULSE_HARMONIC;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &oc);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);

    TIM_Cmd(TIM2, DISABLE);
}

// ==================== TIM2点动模式 ====================
void User_TIM2_Init_Jog(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;
    float freq_hz;
    uint32_t arr;
    uint16_t pulse;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &gpio);

    freq_hz = JOG_SPEED_MMPS * PULSES_PER_MM;
    arr = (uint32_t)(1000000.0f / freq_hz - 1);
    if (arr < 1)
        arr = 1;
    pulse = arr / 2;

    tim.TIM_Period = arr;
    tim.TIM_Prescaler = 71;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = pulse;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &oc);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM_Cmd(TIM2, DISABLE);
}

// ==================== TIM2中断 ====================
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        if (jog_active)
        {
            if (GPIO_ReadOutputDataBit(DIR_PORT, DIR_PIN))
            {
                if (current_position < MAX_TRAVEL_PULSES)
                    current_position++;
                else
                    jog_active = 0;

                if (current_position >= MAX_TRAVEL_PULSES)
                    jog_active = 0;
            }
            else
            {
                if (current_position > 0)
                    current_position--;
                else
                    jog_active = 0;

                if (current_position <= 0)
                    jog_active = 0;
            }

            if (!jog_active)
            {
                TIM_Cmd(TIM2, DISABLE);
                TIM2->CCER &= ~TIM_CCER_CC1E;
                GPIO_ResetBits(PULSE_PORT, PULSE_PIN);
            }
        }
        else if (sending)
        {
            sent_pulses++;
            current_position += target_pulses > 0 ? 1 : -1;
            if (sent_pulses >= (target_pulses > 0 ? target_pulses : -target_pulses))
            {
                TIM_Cmd(TIM2, DISABLE);
                TIM_SetCounter(TIM2, 0);
                sending = 0;
                sent_pulses = 0;
                GPIO_ResetBits(PULSE_PORT, PULSE_PIN);

                if (pending_flag && pending_delta != 0)
                {
                    if (pending_delta > 0)
                        GPIO_SetBits(DIR_PORT, DIR_PIN);
                    else
                        GPIO_ResetBits(DIR_PORT, DIR_PIN);

                    target_pulses = pending_delta;
                    pending_delta = 0;
                    pending_flag = 0;
                    sending = 1;
                    TIM_SetCounter(TIM2, 0);
                    TIM_Cmd(TIM2, ENABLE);
                }
            }
        }
    }
}

// ==================== TIM3 0.5ms时基 ====================
void User_TIM3_Base_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    tim.TIM_Period = 500 - 1;
    tim.TIM_Prescaler = 72 - 1;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tim);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 0);
    TIM_Cmd(TIM3, ENABLE);
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        sys_time_us += 500;
    }
}

// ==================== 发送脉冲（简谐运动） ====================
void User_SendPulsesNonBlock(int32_t pulses)
{
    if (pulses == 0 || jog_active)
        return;

    if (!sending)
    {
        if (pulses > 0)
            GPIO_SetBits(DIR_PORT, DIR_PIN);
        else
            GPIO_ResetBits(DIR_PORT, DIR_PIN);

        target_pulses = pulses;
        sent_pulses = 0;
        sending = 1;
        TIM_SetCounter(TIM2, 0);
        TIM_Cmd(TIM2, ENABLE);
    }
    else
    {
        pending_delta += pulses;
        pending_flag = 1;
    }
}

// ==================== 简谐运动计算 ====================
void User_Motion_Calc(void)
{
    float t;
    float target_float;
    float delta_float;
    int32_t delta_int;

    if (random_disturbance_enable &&
        (uint32_t)(sys_time_us - random_update_time_us) >= RANDOM_UPDATE_US)
    {
        random_update_time_us = sys_time_us;
        random_disturbance_pulses = User_RandomDisturbancePulses();
    }

    t = (uint32_t)(sys_time_us - motion_start_time_us) / 1000000.0f;
    target_float = motion_start_position +
                   (motion_amplitude * PULSES_PER_MM + random_disturbance_pulses) *
                   (1.0f - cosf(2.0f * PI * motion_frequency * t));
    delta_float = target_float - pos_float;
    delta_int = (int32_t)(delta_float + (delta_float >= 0 ? 0.5f : -0.5f));

    if (delta_int != 0)
    {
        pos_float += delta_int;
        User_SendPulsesNonBlock(delta_int);
    }
}

// ==================== 延时 ====================
void User_Delay_ms(uint32_t ms)
{
    uint32_t i;
    uint32_t j;

    for (i = 0; i < ms; i++)
        for (j = 0; j < 8000; j++);
}
