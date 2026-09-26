#include "stm32f10x.h"
#include "HMI.h"

#define HMI_DMA_RX_SIZE     64
#define HMI_RX_RING_SIZE    128
#define HMI_FRAME_QUEUE_SIZE 4

typedef enum
{
    HMI_PARSE_HEADER_55,
    HMI_PARSE_HEADER_AA,
    HMI_PARSE_COMMAND,
    HMI_PARSE_LENGTH,
    HMI_PARSE_DATA
} HMI_ParseState;

static uint8_t dma_rx_buffer[HMI_DMA_RX_SIZE];
static uint8_t rx_ring[HMI_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

static HMI_Frame frame_queue[HMI_FRAME_QUEUE_SIZE];
static uint8_t frame_head;
static uint8_t frame_tail;

static HMI_ParseState parse_state = HMI_PARSE_HEADER_55;
static HMI_Frame parse_frame;
static uint8_t parse_index;

static void HMI_RingPush(uint8_t value)
{
    uint16_t next = rx_head + 1;

    if (next >= HMI_RX_RING_SIZE)
        next = 0;
    if (next == rx_tail)
        return; // 软件缓存已满时丢弃新字节，后续帧头会使解析器重新同步。
    rx_ring[rx_head] = value;
    rx_head = next;
}

static uint8_t HMI_RingPop(uint8_t *value)
{
    uint16_t tail = rx_tail;

    if (tail == rx_head)
        return 0;
    *value = rx_ring[tail];
    tail++;
    if (tail >= HMI_RX_RING_SIZE)
        tail = 0;
    rx_tail = tail;
    return 1;
}

static void HMI_QueueFrame(void)
{
    uint8_t i;
    uint8_t next = frame_head + 1;

    if (next >= HMI_FRAME_QUEUE_SIZE)
        next = 0;
    if (next == frame_tail)
        return;

    frame_queue[frame_head].command = parse_frame.command;
    frame_queue[frame_head].length = parse_frame.length;
    for (i = 0; i < parse_frame.length; i++)
        frame_queue[frame_head].data[i] = parse_frame.data[i];
    frame_head = next;
}

static void HMI_ParseByte(uint8_t value)
{
    switch (parse_state)
    {
        case HMI_PARSE_HEADER_55:
            if (value == 0x55)
                parse_state = HMI_PARSE_HEADER_AA;
            break;

        case HMI_PARSE_HEADER_AA:
            if (value == 0xAA)
                parse_state = HMI_PARSE_COMMAND;
            else if (value != 0x55)
                parse_state = HMI_PARSE_HEADER_55;
            break;

        case HMI_PARSE_COMMAND:
            parse_frame.command = value;
            parse_state = HMI_PARSE_LENGTH;
            break;

        case HMI_PARSE_LENGTH:
            if (value > HMI_MAX_DATA_LENGTH)
            {
                // 非法长度可能来自错位数据，从当前字节重新寻找下一帧头。
                parse_state = value == 0x55 ? HMI_PARSE_HEADER_AA : HMI_PARSE_HEADER_55;
                break;
            }
            parse_frame.length = value;
            parse_index = 0;
            if (value == 0)
            {
                HMI_QueueFrame();
                parse_state = HMI_PARSE_HEADER_55;
            }
            else
                parse_state = HMI_PARSE_DATA;
            break;

        case HMI_PARSE_DATA:
            parse_frame.data[parse_index++] = value;
            if (parse_index >= parse_frame.length)
            {
                HMI_QueueFrame();
                parse_state = HMI_PARSE_HEADER_55;
            }
            break;

        default:
            parse_state = HMI_PARSE_HEADER_55;
            break;
    }
}

void HMI_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    DMA_InitTypeDef dma;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    DMA_DeInit(DMA1_Channel5);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)dma_rx_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = HMI_DMA_RX_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &dma);

    // USART1_RX固定使用DMA1通道5；空闲中断用于确定本批数据的实际长度。
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    NVIC_SetPriority(USART1_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);

    DMA_Cmd(DMA1_Channel5, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void HMI_Process(void)
{
    uint8_t value;

    // 协议解析放在主循环执行，中断只负责尽快搬运DMA接收的数据。
    while (HMI_RingPop(&value))
        HMI_ParseByte(value);
}

uint8_t HMI_GetFrame(HMI_Frame *frame)
{
    uint8_t i;

    if (frame == 0 || frame_tail == frame_head)
        return 0;
    frame->command = frame_queue[frame_tail].command;
    frame->length = frame_queue[frame_tail].length;
    for (i = 0; i < frame->length; i++)
        frame->data[i] = frame_queue[frame_tail].data[i];

    frame_tail++;
    if (frame_tail >= HMI_FRAME_QUEUE_SIZE)
        frame_tail = 0;
    return 1;
}

void HMI_SendFrame(uint8_t command, const uint8_t *data, uint8_t length)
{
    uint8_t i;

    if (length > HMI_MAX_DATA_LENGTH || (length != 0 && data == 0))
        return;

    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, 0x55);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, 0xAA);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, command);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, length);

    for (i = 0; i < length; i++)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART1, data[i]);
    }
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}
}

void USART1_IRQHandler(void)
{
    uint16_t received;
    uint16_t i;
    volatile uint32_t clear_idle;

    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        // IDLE标志必须按“读SR后读DR”的顺序清除，否则中断会重复进入。
        clear_idle = USART1->SR;
        clear_idle = USART1->DR;
        (void)clear_idle;

        DMA_Cmd(DMA1_Channel5, DISABLE);
        received = HMI_DMA_RX_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);
        for (i = 0; i < received; i++)
            HMI_RingPush(dma_rx_buffer[i]);

        // 每次空闲后恢复完整接收长度，保证下一批数据从缓存起始处写入。
        DMA_ClearFlag(DMA1_FLAG_GL5);
        DMA_SetCurrDataCounter(DMA1_Channel5, HMI_DMA_RX_SIZE);
        DMA_Cmd(DMA1_Channel5, ENABLE);
    }
}
