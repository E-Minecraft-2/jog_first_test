#ifndef __HMI_H
#define __HMI_H

#include <stdint.h>

#define HMI_MAX_DATA_LENGTH 16

typedef struct
{
    uint8_t command;
    uint8_t length;
    uint8_t data[HMI_MAX_DATA_LENGTH];
} HMI_Frame;

void HMI_Init(void);
void HMI_Process(void);
uint8_t HMI_GetFrame(HMI_Frame *frame);
void HMI_SendFrame(uint8_t command, const uint8_t *data, uint8_t length);

#endif
