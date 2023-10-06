/** 
	this is for LED dimming
	
**/

#include "main.h"
#include "bsp_led.h"

LED_MEMBER ledMember[LED_NUM];

void BSP_LED_Init(void) //led初始化
{
	LED_MEMBER *ledP;
	uint8_t i;

	ledP = &ledMember[0];
	ledP->GPIOx = LED1_PORT;
	ledP->GPIO_Pin = LED1_PIN;

	for (i = 0; i < LED_NUM; i++)
	{
		ledP = &ledMember[i];
		ledP->mode = (uint8_t)LED_OFF_MODE;
		ledP->tick0 = 0;
		ledP->updateTick = 0;
		ledP->ledPWM.duty0 = 0;
		ledP->ledPWM.duty = 0;
		ledP->ledPWM.period = LED_PWM_BREATHING_PERIOD;
	}
}

void led_control(void)
{
	uint8_t i;
	LED_MEMBER *ledP;
	for (i = 0; i < LED_NUM; i++)
	{
		ledP = &ledMember[i];
		switch (ledP->mode)
		{
		case LED_OFF_MODE:
			BSP_LED_Off(ledP);
			break;
		case LED_1_MODE:
			led_dim1_mode(ledP);
			break;
		case LED_2_MODE:
			led_dim2_mode(ledP);
			break;
		case LED_3_MODE:
			led_dim3_mode(ledP);
			break;
		case LED_ON_MODE:
			BSP_LED_On(ledP);
			break;
		case LED_PWM_MODE:
			led_PWM_mode(ledP);
			break;
		case LED_BREATHING_MODE:
			led_breathing_mode(ledP);
			break;
		case LED_UNDO_MODE:
			break;
		default:
			BSP_LED_Off(ledP);
			break;
		}
	}
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *            @arg  LED3
  * @retval None
  */
void BSP_LED_On(LED_MEMBER *ledP)
{
	HAL_GPIO_WritePin(ledP->GPIOx, ledP->GPIO_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Turns selected LED Off. 
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *            @arg  LED3
  * @retval None
  */
void BSP_LED_Off(LED_MEMBER *ledP)
{
	HAL_GPIO_WritePin(ledP->GPIOx, ledP->GPIO_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *            @arg  LED3
  * @retval None
  */
void BSP_LED_Toggle(LED_MEMBER *ledP)
{
	HAL_GPIO_TogglePin(ledP->GPIOx, ledP->GPIO_Pin);
}

void BSP_LED_On2(uint8_t index)
{
	if (index < LED_NUM)
		HAL_GPIO_WritePin(ledMember[index].GPIOx, ledMember[index].GPIO_Pin, GPIO_PIN_RESET);
}

void BSP_LED_Off2(uint8_t index)
{
	if (index < LED_NUM)
		HAL_GPIO_WritePin(ledMember[index].GPIOx, ledMember[index].GPIO_Pin, GPIO_PIN_SET);
}

void BSP_LED_Toggle2(uint8_t index)
{
	if (index < LED_NUM)
		HAL_GPIO_TogglePin(ledMember[index].GPIOx, ledMember[index].GPIO_Pin);
}

void led_dim1_mode(LED_MEMBER *ledP)
{
	uint32_t led_tick, diff_tick;
	led_tick = HAL_GetTick();
	if (led_tick < ledP->tick0)
		ledP->tick0 = led_tick;
	diff_tick = led_tick - ledP->tick0;

	// updateTick的意义在于规定更新的时间间隔
	if (ledP->updateTick != diff_tick)
	{
		ledP->updateTick = diff_tick;

		if (diff_tick < 700)
		{
			BSP_LED_On(ledP);
		}
		else
		{
			BSP_LED_Off(ledP);
			if (diff_tick >= 1300)
				ledP->tick0 = led_tick;
		}
	}
}

void led_dim2_mode(LED_MEMBER *ledP)
{
	uint32_t led_tick, diff_tick;
	led_tick = HAL_GetTick();
	if (led_tick < ledP->tick0)
		ledP->tick0 = led_tick;
	diff_tick = led_tick - ledP->tick0;

	// updateTick的意义在于规定更新的时间间隔
	if (ledP->updateTick != diff_tick)
	{
		ledP->updateTick = diff_tick;

		if (diff_tick < 50)
		{
			BSP_LED_On(ledP);
		}
		else if (diff_tick < 150)
		{
			BSP_LED_Off(ledP);
		}
		else if (diff_tick < 200)
		{
			BSP_LED_On(ledP);
		}
		else if (diff_tick < 1000)
		{
			BSP_LED_Off(ledP);
		}
		else
		{
			ledP->tick0 = led_tick;
		}
	}
}

void led_dim3_mode(LED_MEMBER *ledP)
{
	uint32_t led_tick, diff_tick;
	led_tick = HAL_GetTick();
	if (led_tick < ledP->tick0)
		ledP->tick0 = led_tick;
	diff_tick = led_tick - ledP->tick0;

	// updateTick的意义在于规定更新的时间间隔
	if (ledP->updateTick != diff_tick)
	{
		ledP->updateTick = diff_tick;

		if (diff_tick < 100)
		{
			BSP_LED_On(ledP);
		}
		else
		{
			BSP_LED_Off(ledP);
			if (diff_tick >= 3000)
				ledP->tick0 = led_tick;
		}
	}
}

void led_PWM_mode(LED_MEMBER *ledP)
{
	uint32_t led_tick, diff_tick;

	led_tick = HAL_GetTick();
	if (led_tick < ledP->tick0)
		ledP->tick0 = led_tick;
	diff_tick = led_tick - ledP->tick0;

	// one PWM cycle finished
	if (diff_tick >= ledP->ledPWM.period)
		ledP->tick0 = led_tick;

	// updateTick的意义在于规定更新的时间间隔
	if (ledP->updateTick != diff_tick)
	{
		ledP->updateTick = diff_tick;

		if ((diff_tick >= ledP->ledPWM.duty0) && (diff_tick < ledP->ledPWM.duty))
			BSP_LED_On(ledP);
		else
			BSP_LED_Off(ledP);
	}
}

void led_breathing_mode(LED_MEMBER *ledP)
{
	uint32_t led_tick, diff_tick, breath_diff_tick;

	led_tick = HAL_GetTick();

	// 8888888888888888888888888888888888888888888888888888
	// update PWM Part
	// 8888888888888888888888888888888888888888888888888888
	if (led_tick < ledP->tick0)
		ledP->tick0 = led_tick;
	diff_tick = led_tick - ledP->tick0;
	// one PWM cycle finished
	if (diff_tick >= ledP->ledPWM.period)
		ledP->tick0 = led_tick;

	// updateTick的意义在于规定更新的时间间隔
	if (ledP->updateTick != diff_tick)
	{
		ledP->updateTick = diff_tick;

		if ((diff_tick >= ledP->ledPWM.duty0) && (diff_tick < ledP->ledPWM.duty))
			BSP_LED_On(ledP);
		else
			BSP_LED_Off(ledP);
	}

	// 8888888888888888888888888888888888888888888888888888
	// update Breathing Part
	// 8888888888888888888888888888888888888888888888888888
	if (led_tick < ledP->ledBreathing.breathing_tick0)
		ledP->ledBreathing.breathing_tick0 = led_tick;
	breath_diff_tick = led_tick - ledP->ledBreathing.breathing_tick0;
	// one Breathing step finished
	if (breath_diff_tick >= ledP->ledBreathing.breathing_step_period)
	{
		ledP->ledBreathing.breathing_tick0 = led_tick;

		if (ledP->ledBreathing.breathing_dir == 0)
		{
			if (ledP->ledPWM.duty < ledP->ledPWM.period)
			{
				ledP->ledPWM.duty++;
			}
			else
			{
				ledP->ledBreathing.breathing_dir = 1;
			}
		}
		else
		{
			if (ledP->ledPWM.duty > 0)
			{
				ledP->ledPWM.duty--;
			}
			else
			{
				ledP->ledBreathing.breathing_dir = 0;
			}
		}
	}
}

void led_mode_set(LED_MEMBER *ledP, uint8_t mode)
{
	ledP->mode = mode;
}

uint8_t led_mode_get(LED_MEMBER *ledP)
{
	return ledP->mode;
}

void led_mode_set2(uint8_t index, uint8_t mode)
{
	if (index < LED_NUM)
		ledMember[index].mode = mode;
}

uint8_t led_mode_get2(uint8_t index)
{
	if (index < LED_NUM)
		return ledMember[index].mode;
	else
		return 0;
}

void led_pwm_mode_config(LED_MEMBER *ledP, LED_PWM ledpwm)
{
	ledP->ledPWM.duty0 = ledpwm.duty0;
	ledP->ledPWM.duty = ledpwm.duty;
	ledP->ledPWM.period = ledpwm.period;
}

void led_pwm_mode_config_duty(uint8_t index, float duty)
{
	uint32_t period = LED_PWM_DIMMING_PERIOD;
	LED_PWM ledpwm;
	ledpwm.period = period;
	ledpwm.duty0 = 0;
	ledpwm.duty = (uint32_t)(duty * period);

	ledMember[index].ledPWM.duty0 = ledpwm.duty0;
	ledMember[index].ledPWM.duty = ledpwm.duty;
	ledMember[index].ledPWM.period = ledpwm.period;
}

void led_pwm_mode_config_breathing(uint8_t index)
{
	//ledMember[index].mode = LED_BREATHING_MODE;
	ledMember[index].ledPWM.period = LED_PWM_BREATHING_PERIOD;
	ledMember[index].ledPWM.duty0 = 0;
	ledMember[index].ledPWM.duty = 0;

	ledMember[index].ledBreathing.breathing_dir = 0;
	ledMember[index].ledBreathing.breathing_step_period = LED_BREATHING_STEP_PERIOD;
	ledMember[index].ledBreathing.breathing_tick0 = HAL_GetTick();
}

void led_tick_reset(LED_MEMBER *ledP)
{
	ledP->tick0 = HAL_GetTick();
}

void horse_race_lamp(uint16_t ledMask, uint32_t period, uint8_t num)
{
	// ledMask -> 每一位代表一个跑马灯
	// period -> 跑马灯周期
	// num -> 跑马灯数量
	if (num == 0)
		return;

	LED_MEMBER *ledP;

	uint32_t duty = period / num;
	uint8_t i;
	LED_PWM ledpwm;
	ledpwm.period = period;
	ledpwm.duty0 = 0;

	for (i = 0; i < LED_NUM; i++)
	{
		if ((0x0001 & (ledMask >> i)) != 0x00)
		{
			ledpwm.duty = ledpwm.duty0 + duty;
			ledP = &ledMember[i];
			led_pwm_mode_config(ledP, ledpwm);
			led_mode_set(ledP, LED_PWM_MODE);
			led_tick_reset(ledP);
			ledpwm.duty0 += duty;
		}
	}
}

void vTaskLedBreathing(void *pvParameters)
{
	BSP_LED_Init();

	led_mode_set2(0, LED_1_MODE);

	while (1)
	{		
		led_control();

		//BSP_LED_Toggle2(0);

		delay(10);
	}

	// 如果任务不是永久性的需要调用 vTaskDelete 删除任务
	// vTaskDelete(NULL);
}
