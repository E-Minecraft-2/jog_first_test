#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_Stop(void);
void Motor_StartMotion(uint32_t now_us);
void Motor_ReturnToStart(void);
void Motor_ToggleRandom(uint32_t now_us);
void Motor_SetRandomEnabled(uint8_t enabled, uint32_t now_us);
void Motor_ProcessJog(uint8_t neg_pressed, uint8_t pos_pressed);
void Motor_JogStep(int8_t direction);
void Motor_Update(uint32_t now_us);
void Motor_ChangeFrequency(int8_t direction);
void Motor_ChangeAmplitude(int8_t direction);
void Motor_ChangeInterference(int8_t direction);
void Motor_ChangeJogAmplitude(int8_t direction);
void Motor_SetFrequencyX100(uint16_t frequency_x100);
void Motor_SetAmplitudeX100(uint16_t amplitude_x100);
void Motor_SetInterference(uint8_t interference);
void Motor_SetJogAmplitudeMm(uint16_t amplitude_mm);
uint8_t Motor_IsMotionEnabled(void);
uint8_t Motor_IsJogActive(void);
uint8_t Motor_IsReturnActive(void);
uint8_t Motor_IsSending(void);
uint8_t Motor_HasLimitError(void);
uint8_t Motor_IsRandomEnabled(void);
int32_t Motor_GetPositionMm(void);
int32_t Motor_GetPositionX100Mm(void);
uint16_t Motor_GetFrequencyX100(void);
uint16_t Motor_GetAmplitudeMm(void);
uint16_t Motor_GetAmplitudeX100(void);
uint8_t Motor_GetInterference(void);
uint16_t Motor_GetJogAmplitudeMm(void);

#endif
