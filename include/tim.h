#ifndef tim_h
#define tim_h

#include "stm32f4xx_hal.h"

void tim_init(void);

void PWM_duty_set(uint8_t index, float duty);
float PWM_duty_get(uint8_t index);

#endif