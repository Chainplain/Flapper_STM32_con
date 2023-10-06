/**

**/

#ifndef _BSP_LED_H
#define _BSP_LED_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define LED_NUM 1
#define LED_PWM_DIMMING_PERIOD 10
#define LED_PWM_BREATHING_PERIOD 10
#define LED_BREATHING_STEP_PERIOD 200

enum 
{
	LED_OFF_MODE = 0,
	LED_ON_MODE = 3,
	LED_1_MODE = 1,
	LED_2_MODE = 2,
	LED_PWM_MODE = 4,
	LED_BREATHING_MODE = 5,
	LED_3_MODE,
	LED_UNDO_MODE,
};

typedef struct
{
	uint32_t breathing_step_period;
	uint8_t breathing_dir;
	uint32_t breathing_tick0;
}LED_BREATHING;

typedef struct
{
	uint32_t duty0;
	uint32_t duty;
	uint32_t period;
}LED_PWM;

typedef struct
{
	GPIO_TypeDef* GPIOx;
	uint16_t			GPIO_Pin;
	uint8_t				mode;
	uint32_t 			tick0;
	uint32_t 			updateTick;
	LED_PWM				ledPWM;
	LED_BREATHING ledBreathing;
}LED_MEMBER;

//#define LED_PORT 	GPIOB
//#define LED_PIN		GPIO_PIN_4
//#define LED_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE()

void BSP_LED_Init(void);
void BSP_LED_On(LED_MEMBER* ledP);
void BSP_LED_Off(LED_MEMBER* ledP);
void BSP_LED_Toggle(LED_MEMBER* ledP);
void BSP_LED_On2(uint8_t index);
void BSP_LED_Off2(uint8_t index);
void BSP_LED_Toggle2(uint8_t index);

void led_dim1_mode(LED_MEMBER* ledP);
void led_dim2_mode(LED_MEMBER* ledP);
void led_dim3_mode(LED_MEMBER* ledP);
void led_PWM_mode(LED_MEMBER* ledP);
void led_breathing_mode(LED_MEMBER* ledP);

void led_mode_set(LED_MEMBER* ledP, uint8_t mode);
uint8_t led_mode_get(LED_MEMBER* ledP);
void led_mode_set2(uint8_t index, uint8_t mode);
uint8_t led_mode_get2(uint8_t index);
void led_pwm_mode_config(LED_MEMBER* ledP, LED_PWM ledpwm);
void led_pwm_mode_config_duty(uint8_t index, float duty);
void led_pwm_mode_config_breathing(uint8_t index);
void led_tick_reset(LED_MEMBER* ledP);
void horse_race_lamp(uint16_t ledMask, uint32_t period, uint8_t num);

void led_control(void);

void vTaskLedBreathing(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif
