/**

**/

#ifndef _SYSTEM_CLOCK_RCC_H
#define _SYSTEM_CLOCK_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"

void SystemClock_Init(void);

void SystemClockHSI_Config(void);
void SystemClockHSE_Config(void);
void SystemClockHSE_PLL_Config(void);
void SystemClockHSI_PLL_Config(void);

void SystemClock_HSI_4MHz_Config(void);
void SystemClock_HSE_4MHz_Config(void);
void SystemClock_HSI_32MHz_Config(void);

void gpio_power_save(void);


#ifdef __cplusplus
}
#endif

#endif
