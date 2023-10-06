

#ifndef _ADC_H
#define _ADC_H

#include "stm32f4xx_hal.h"

#define ADC_OUT_CHANNELS 7

struct sAdcVal
{
    float vb;
    //float vo[4];
    float io[7];
};

#define ADC_HANDLE ADC1
#define ADC_CLOCK_ENABLE() __HAL_RCC_ADC1_CLK_ENABLE()
#define ADC_CLOCK_DISABLE() __HAL_RCC_ADC1_CLK_DISABLE()
#define ADC_TRIGGER_TIMER TIM8
#define ADC_TRIGGER_TIMER_CLOCK_ENABLE() __HAL_RCC_TIM8_CLK_ENABLE()
#define ADC_TRIGGER_MODE ADC_EXTERNALTRIGCONV_T8_TRGO

#define ADC_SAMPLE_FREQUENCY 50e4f
#define ADC_NUMBERS 9
#define ADC_BUFFER_LEN 18

void ADC_init(void);
void ADC_MspInit(void);
void ADC_PPInit(void);

// *********** DMA **********************************************
#define DMA_ADC 1
#define DMA_ADC_CLOCK_ENABLE __HAL_RCC_DMA2_CLK_ENABLE()
#define DMA_ADC_CLOCK_DISABLE __HAL_RCC_DMA2_CLK_DISABLE()

#define DMA_ADC_Channel DMA_CHANNEL_0
#define DMA_ADC_Stream DMA2_Stream0

#define DMA_ADC_IRQn DMA2_Stream0_IRQn
#define DMA_ADC_IRQHandler DMA2_Stream0_IRQHandler

#define DMA_ADC_ISR_FLAG_ALL_CLEAR DMA2->LIFCR |= 0x3F; // clear stream 0 all interrupt flag
#define DMA_ADC_ISR_VALUE (DMA2->LISR & 0x3F)           // get stream 0 all interrupt flag
#define DMA_ADC_COMPLETE_MASK 0x20
#define DMA_ADC_ERROR_MASK 0x08

void ADC_DMA_callback(void);
float ADC_VBAT_Get(void);
float ADC_Temperature_Get(void);
sAdcVal ADC_FB_Get(void);

extern void (*adc_received_callback) (sAdcVal *);

#endif
