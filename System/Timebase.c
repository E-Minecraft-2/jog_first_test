#include "stm32f10x.h"
#include "Timebase.h"

static volatile uint32_t sys_time_us; // 系统时间，单位：微秒

void Timebase_Init(void)
{
    // TIM3每0.5 ms产生一次更新中断，作为全工程非阻塞时基
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

uint32_t Timebase_GetUs(void)
{
    return sys_time_us;
}

void TIM3_IRQHandler(void)
{
    // TIM3中断优先级高于TIM2，保证按键消抖和运动时间基准稳定
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        sys_time_us += 500;
    }
}
