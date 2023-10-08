#pragma once

#ifndef control_h
#define control_h

#include <stm32f4xx_hal.h>
#include "adc.h"

void vTaskControl(void *pvParameters);

void adc_data_received(sAdcVal* adc_val);

#endif