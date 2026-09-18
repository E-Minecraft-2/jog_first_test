#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

typedef enum
{
    KEY_START,
    KEY_NEG,
    KEY_POS,
    KEY_RANDOM,
    KEY_MODE,
    KEY_COUNT
} Key_Id;

void Key_Init(void);
void Key_Scan(uint32_t now_us);
void Key_ClearEvents(void);
uint8_t Key_GetEvent(Key_Id key);
uint8_t Key_IsPressed(Key_Id key);

#endif
