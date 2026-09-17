#include "stm32f10x.h"
#include "Delay.h"

void Key_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 内部上拉，低电平有效
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 非阻塞按键扫描，返回键码（按下时返回1~5，否则返回0）
uint8_t Key_GetNum(void)
{
    static uint8_t last_state[5] = {1,1,1,1,1};   // 上一次状态（默认高）
    uint8_t pin_list[5] = {GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6};
    uint8_t i, current;
    for (i = 0; i < 5; i++) {
        current = GPIO_ReadInputDataBit(GPIOA, pin_list[i]);
        if (last_state[i] == 1 && current == 0) {  // 下降沿
            Delay_ms(20);                          // 消抖
            if (GPIO_ReadInputDataBit(GPIOA, pin_list[i]) == 0) {
                last_state[i] = 0;
                return (i+1);                      // 返回键码 1~5
            }
        }
        last_state[i] = current;
    }
    return 0;
}
