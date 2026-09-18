#include "stm32f10x.h"
#include "Motor.h"
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
#define PULSES_PER_REV        2000UL                              // 每转脉冲数（含细分）
#define LEAD_SCREW_PITCH      20.0f                               // 滚珠丝杆导程，单位：mm
#define PULSES_PER_MM         (PULSES_PER_REV / LEAD_SCREW_PITCH) // 每毫米脉冲数
#define MAX_TRAVEL_MM         950.0f // 电缸最大运行长度，单位：mm
#define MAX_TRAVEL_PULSES     ((int32_t)(MAX_TRAVEL_MM * PULSES_PER_MM)) // 电缸最大运行脉冲数
#define RANDOM_DISTURBANCE_PULSES ((int32_t)(RANDOM_DISTURBANCE_MM * PULSES_PER_MM)) // 随机扰动幅值，单位：脉冲

// TIM2 PWM载波频率，简谐运动模式使用
#define PWM_PULSE_FREQ        100000UL // PWM脉冲频率，单位：Hz
#define ARR_VALUE_HARMONIC    (1000000 / PWM_PULSE_FREQ - 1) // TIM2自动重装载值
#define HALF_PULSE_HARMONIC   ((ARR_VALUE_HARMONIC + 1) / 2) // 50%占空比比较匹配值
#define PI                    3.1415926f // 圆周率

// ==================== 电机硬件引脚 ====================
#define PULSE_PIN             GPIO_Pin_0  // PA0（TIM2_CH1）脉冲输出
#define PULSE_PORT            GPIOA
#define DIR_PIN               GPIO_Pin_10 // PB10 方向输出
#define DIR_PORT              GPIOB

// 由TIM2中断更新的运动状态
static volatile int32_t current_position; // 当前脉冲位置
static float pos_float;                   // 用于简谐运动计算的浮点位置
static volatile int32_t target_pulses;    // 当前批次目标脉冲数
static volatile uint32_t sent_pulses;     // 当前批次已发送脉冲数
static volatile uint8_t sending;          // 正在发送非阻塞脉冲
static volatile int32_t pending_delta;    // 下一批待发送脉冲增量
static volatile uint8_t pending_flag;     // 存在待发送脉冲增量
static volatile uint8_t motion_enable;    // 简谐运动已启动
static volatile uint8_t jog_active;       // 点动正在执行
static uint8_t return_active;             // 正在返回简谐运动起点
static float motion_frequency = MOTION_FREQUENCY;     // 简谐运动频率，单位：Hz
static float motion_amplitude = MOTION_AMPLITUDE_MM;  // 简谐运动幅值，单位：mm
static int32_t motion_start_position;                 // 简谐运动起点脉冲位置
static uint32_t motion_start_time_us;                 // 简谐运动起点时间，单位：微秒
static uint8_t motion_limit_error;                    // 简谐运动起点或幅值超出行程限制
static uint8_t random_disturbance_enable;             // 随机扰动使能
static int32_t random_disturbance_pulses;             // 随机扰动幅值，单位：脉冲
static uint32_t random_update_time_us;                // 随机扰动上次更新时间，单位：微秒
static uint32_t random_state = 0x13579BDFUL;          // 随机数生成器状态

// PA0为复用输出，必须由TIM2强制输出低电平，不能用GPIO复位代替。
static void Motor_StopPulseOutput(void)
{
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM2, DISABLE);
    TIM_ForcedOC1Config(TIM2, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM2, TIM_Channel_1, TIM_CCx_Enable);
    TIM_SetCounter(TIM2, 0);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
}

// 状态和方向准备好后才启动。PWM2从低电平开始，更新事件对应脉冲结束。
static void Motor_StartPulseOutput(void)
{
    TIM_SetCounter(TIM2, 0);
    TIM_SelectOCxM(TIM2, TIM_Channel_1, TIM_OCMode_PWM2);
    TIM_CCxCmd(TIM2, TIM_Channel_1, TIM_CCx_Enable);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

// 配置TIM2为固定100 kHz脉冲频率，供简谐运动使用
static void Motor_InitHarmonic(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    Motor_StopPulseOutput();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = PULSE_PIN;
    GPIO_Init(PULSE_PORT, &gpio);
    tim.TIM_Period = ARR_VALUE_HARMONIC;
    tim.TIM_Prescaler = 71;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);
    oc.TIM_OCMode = TIM_OCMode_Timing; // 配置期间保持此前强制的低电平
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = HALF_PULSE_HARMONIC;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &oc);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    // 装载ARR/CCR预装载值后清除软件更新事件，初始化不参与脉冲计数。
    TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
    Motor_StopPulseOutput();
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief 初始化Jog模式
 */
static void Motor_InitJog(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;
    uint32_t arr = (uint32_t)(1000000.0f / (JOG_SPEED_MMPS * PULSES_PER_MM) - 1);

    if (arr < 1)
        arr = 1;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    Motor_StopPulseOutput();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = PULSE_PIN;
    GPIO_Init(PULSE_PORT, &gpio);
    tim.TIM_Period = arr;
    tim.TIM_Prescaler = 71;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);
    oc.TIM_OCMode = TIM_OCMode_Timing; // 配置期间保持此前强制的低电平
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = (arr + 1) / 2;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &oc);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
    Motor_StopPulseOutput();
    NVIC_EnableIRQ(TIM2_IRQn);
}

// 判断给定起点及当前幅值是否能满足全程行程限制
static uint8_t Motor_FitsTravel(int32_t start_position, uint8_t random_enable)
{
    int32_t amplitude_pulses = (int32_t)(motion_amplitude * PULSES_PER_MM);

    if (random_enable)
        amplitude_pulses += RANDOM_DISTURBANCE_PULSES;
    return start_position >= 0 && start_position <= MAX_TRAVEL_PULSES - 2 * amplitude_pulses;
}

static int32_t Motor_RandomDisturbancePulses(void)
{
    random_state = random_state * 1664525UL + 1013904223UL;
    return (int32_t)(random_state % (2 * RANDOM_DISTURBANCE_PULSES + 1)) - RANDOM_DISTURBANCE_PULSES;
}

// 发送脉冲；当前批次未完成时将增量累积到下一批
static void Motor_SendPulses(int32_t pulses)
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
        Motor_StartPulseOutput();
    }
    else
    {
        pending_delta += pulses;
        pending_flag = 1;
    }
}

void Motor_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = DIR_PIN;
    GPIO_Init(DIR_PORT, &gpio);
    Motor_InitHarmonic();
    GPIO_SetBits(DIR_PORT, DIR_PIN);
}

/**
 * @brief 停止所有运动
 */
void Motor_Stop(void)
{
    Motor_StopPulseOutput();
    motion_enable = 0;
    jog_active = 0;
    return_active = 0;
    sending = 0;
    sent_pulses = 0;
    pending_delta = 0;
    pending_flag = 0;
}
/**
 * @brief 开始简谐运动
 * 
 * @param now_us 
 */
void Motor_StartMotion(uint32_t now_us)
{
    if (!Motor_FitsTravel(current_position, random_disturbance_enable))
    {
        motion_limit_error = 1;
        return;
    }
    motion_enable = 1;
    motion_limit_error = 0;
    motion_start_position = current_position;
    motion_start_time_us = now_us;
    random_disturbance_pulses = 0;
    random_update_time_us = now_us;
    pos_float = (float)current_position;
    Motor_InitHarmonic();
}
/**
 * @brief 返回起点
 */
void Motor_ReturnToStart(void)
{
    Motor_Stop();
    if (current_position != motion_start_position)
    {
        Motor_InitJog();
        return_active = 1;
        Motor_SendPulses(motion_start_position - current_position);
    }
}
/**
 * @brief 切换随机扰动使能状态
 * 
 * @param now_us 
 */
void Motor_ToggleRandom(uint32_t now_us)
{
    if (random_disturbance_enable)
    {
        random_disturbance_enable = 0;
        random_disturbance_pulses = 0;
    }
    else if (Motor_FitsTravel(motion_enable ? motion_start_position : current_position, 1))
    {
        random_disturbance_enable = 1;
        motion_limit_error = 0;
        random_disturbance_pulses = 0;
        random_update_time_us = now_us;
    }
    else
        motion_limit_error = 1;
}

/**
 * @brief 处理Jog模式
 * 
 * @param neg_pressed 
 * @param pos_pressed 
 */
void Motor_ProcessJog(uint8_t neg_pressed, uint8_t pos_pressed)
{
    if (motion_enable || return_active)
        return;
    if (neg_pressed && pos_pressed)
    {
        if (jog_active)
        {
            Motor_StopPulseOutput();
            jog_active = 0;
        }
        return;
    }
    if (neg_pressed || pos_pressed)
    {
        if (!jog_active)
        {
            // 已到限位且仍按住同方向键时，不重复初始化定时器。
            if ((neg_pressed && current_position <= 0) ||
                (pos_pressed && current_position >= MAX_TRAVEL_PULSES))
                return;
            Motor_InitJog();
            pending_delta = 0;
            pending_flag = 0;
            sending = 0;
            sent_pulses = 0;
            if (neg_pressed)
                GPIO_ResetBits(DIR_PORT, DIR_PIN);
            else
                GPIO_SetBits(DIR_PORT, DIR_PIN);
            if ((neg_pressed && current_position > 0) || (pos_pressed && current_position < MAX_TRAVEL_PULSES))
            {
                jog_active = 1;
                Motor_StartPulseOutput();
            }
            else
                jog_active = 0;
        }
        else if (neg_pressed)
            GPIO_ResetBits(DIR_PORT, DIR_PIN);
        else
            GPIO_SetBits(DIR_PORT, DIR_PIN);
    }
    else if (jog_active)
    {
        Motor_StopPulseOutput();
        jog_active = 0;
    }
}
/** @brief 更新电机状态
 * 
 * @param now_us 
 */
void Motor_Update(uint32_t now_us)
{
    float t;
    float target_float;
    float delta_float;
    int32_t delta_int;

    if (random_disturbance_enable && (uint32_t)(now_us - random_update_time_us) >= RANDOM_UPDATE_US)
    {
        random_update_time_us = now_us;
        random_disturbance_pulses = Motor_RandomDisturbancePulses();
    }
    t = (uint32_t)(now_us - motion_start_time_us) / 1000000.0f;
    target_float = motion_start_position +
        (motion_amplitude * PULSES_PER_MM + random_disturbance_pulses) *
        (1.0f - cosf(2.0f * PI * motion_frequency * t));
    delta_float = target_float - pos_float;
    delta_int = (int32_t)(delta_float + (delta_float >= 0 ? 0.5f : -0.5f));
    if (delta_int != 0)
    {
        pos_float += delta_int;
        Motor_SendPulses(delta_int);
    }
}
/** @brief 更新电机状态
 * 
 * @param now_us 
 */
void Motor_Update(uint32_t now_us)
{
    float t;
    float target_float;
    float delta_float;
    int32_t delta_int;

    if (random_disturbance_enable && (uint32_t)(now_us - random_update_time_us) >= RANDOM_UPDATE_US)
    {
        random_update_time_us = now_us;
        random_disturbance_pulses = Motor_RandomDisturbancePulses();
    }
    t = (uint32_t)(now_us - motion_start_time_us) / 1000000.0f;
    target_float = motion_start_position +
        (motion_amplitude * PULSES_PER_MM + random_disturbance_pulses) *
        (1.0f - cosf(2.0f * PI * motion_frequency * t));
    delta_float = target_float - pos_float;
    delta_int = (int32_t)(delta_float + (delta_float >= 0 ? 0.5f : -0.5f));
    if (delta_int != 0)
    {
        pos_float += delta_int;
        Motor_SendPulses(delta_int);
    }
}
/** @brief 改变简谐运动频率
 * 
 * @param direction 
 */
void Motor_ChangeFrequency(int8_t direction)
{
    if (direction < 0)
        motion_frequency = motion_frequency > MOTION_FREQUENCY_MIN + MOTION_FREQUENCY_STEP ?
            motion_frequency - MOTION_FREQUENCY_STEP : MOTION_FREQUENCY_MIN;
    else
        motion_frequency = motion_frequency < MOTION_FREQUENCY_MAX - MOTION_FREQUENCY_STEP ?
            motion_frequency + MOTION_FREQUENCY_STEP : MOTION_FREQUENCY_MAX;
}
/** @brief 改变简谐运动振幅
 * 
 * @param direction 
 */
void Motor_ChangeAmplitude(int8_t direction)
{
    if (direction < 0)
        motion_amplitude = motion_amplitude > MOTION_AMPLITUDE_MIN + MOTION_AMPLITUDE_STEP ?
            motion_amplitude - MOTION_AMPLITUDE_STEP : MOTION_AMPLITUDE_MIN;
    else
        motion_amplitude = motion_amplitude < MOTION_AMPLITUDE_MAX - MOTION_AMPLITUDE_STEP ?
            motion_amplitude + MOTION_AMPLITUDE_STEP : MOTION_AMPLITUDE_MAX;
}

uint8_t Motor_IsMotionEnabled(void) { return motion_enable; } // 检查简谐运动是否启用
uint8_t Motor_IsJogActive(void) { return jog_active; } // 检查Jog模式是否激活
uint8_t Motor_IsReturnActive(void) { return return_active; } // 检查返回起点模式是否激活
uint8_t Motor_IsSending(void) { return sending; } // 检查是否正在发送脉冲
uint8_t Motor_HasLimitError(void) { return motion_limit_error; } // 检查是否发生限位错误
uint8_t Motor_IsRandomEnabled(void) { return random_disturbance_enable; } // 检查随机扰动是否启用
int32_t Motor_GetPositionMm(void) { return current_position / PULSES_PER_MM; }
uint16_t Motor_GetFrequencyX100(void) { return (uint16_t)(motion_frequency * 100.0f + 0.5f); }
uint16_t Motor_GetAmplitudeMm(void) { return (uint16_t)(motion_amplitude + 0.5f); }

void TIM2_IRQHandler(void)
{
    // 脉冲计数、点动限位与非阻塞脉冲发送都在TIM2更新中断中完成
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
                Motor_StopPulseOutput();
            }
        }
        else if (sending)
        {
            sent_pulses++;
            current_position += target_pulses > 0 ? 1 : -1;
            if (sent_pulses >= (target_pulses > 0 ? target_pulses : -target_pulses))
            {
                Motor_StopPulseOutput();
                sending = 0;
                sent_pulses = 0;
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
                    Motor_StartPulseOutput();
                }
            }
        }
    }
}
