#ifndef pinConfig_h
#define pinConfig_h

#ifdef __cplusplus
extern "C"
{
#endif

#include <stm32f4xx_hal.h>
#include "pinTypes.h"

void pinMode(PinName pinName, PinMode pinMode);
GPIO_PinState digitalRead(PinName pinName);
void digitalWrite(PinName pinName, uint8_t PinState);
void digitalToggle(PinName pinName);

bool GPIO_subscribe(GPIO_TypeDef *GPIO_port, uint16_t GPIO_pin, uint32_t pull_up_down, void (*callback)(void *), void *ctx);
void GPIO_unsubscribe(GPIO_TypeDef *GPIO_port, uint16_t GPIO_pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin);

#ifdef __cplusplus
}
#endif

#endif