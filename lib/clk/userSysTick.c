/** 
	basic PWM setting
	
**/

#include "userSysTick.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h" 

TIM_HandleTypeDef htimbase;

// ================================================================
// ================================================================
// ================================================================
// -*********** 接下来的都是 重写 的函数 *********************
// ================================================================
// ================================================================
// ================================================================
// re-write function
//
// 原始程序在 stm32xxx_hal.c

/**
 * @brief 重写 HAL_InitTick， 原始程序在 stm32xxx_hal.c
 * @param TickPriority 采用了freeRTOS，此参数无效，优先级不再是系统默认的了
 * @return
 * */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef clkconfig;
  uint32_t uwTimclock = 0;
  uint32_t uwPrescalerValue = 0;
  uint32_t pFLatency;

  // Configure the TIM IRQ priority
  // 重新设置优先级
  // 不能再使用systick的优先级了
  // case 1: 如果采用其他中断来作为时间间隔做计算（比如PID，速度等的计算），不需要非常精确的时间计数，把这个中断优先级放到很低的位置
  uint32_t TickPriorityNew = configLIBRARY_LOWEST_INTERRUPT_PRIORITY - 1;
  // case 2: 如果采用 timestamp 作为时间管理的话， 这里为了非常精确的时间，此中断时RTOS无法影响的，具有非常搞的优先级
  //uint32_t TickPriorityNew = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1; 

  HAL_NVIC_SetPriority(TIMBASE_IRQn, TickPriorityNew, 0);
  // Enable the TIM global Interrupt
  HAL_NVIC_EnableIRQ(TIMBASE_IRQn);

  // Enable TIM14 clock
  __HAL_RCC_TIM_TIMEBASE_CLK_ENABLE();

  // Get clock configuration
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  // Compute TIM clock
  // 乘以2是因为F4芯片当APB1!=1时，需要乘以2
  uwTimclock = tim_CK_INT_get(TIM_TIMEBASE);

  // Compute the prescaler value to have TIM counter clock equal to 1MHz
  uwPrescalerValue = (uint32_t)((uwTimclock / 1000000) - 1);

  // Initialize TIM
  htimbase.Instance = TIM_TIMEBASE;

  //  Initialize TIMx peripheral as follow:
  //+ Period = [(TIM14CLK/1000) - 1]. to have a (1/1000) s time base.
  //+ Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  //+ ClockDivision = 0
  //+ Counter direction = Up
  // 可以调用 HAL_SetTickFreq 来改变 滴答时钟的周期
  //htimbase.Init.Period = (1000 / uwTickFreq) - 1;
  htimbase.Init.Period = 1000 - 1; // 1kHz/1ms
  htimbase.Init.Prescaler = uwPrescalerValue;
  htimbase.Init.ClockDivision = 0;
  htimbase.Init.CounterMode = TIM_COUNTERMODE_UP;
  if (HAL_TIM_Base_Init(&htimbase) == HAL_OK)
  {
    // Start the TIM time Base generation in interrupt mode
    return HAL_TIM_Base_Start_IT(&htimbase);
  }

  // Return function status
  return HAL_ERROR;
}

void HAL_SuspendTick(void)
{
  // Disable TIM8 update Interrupt
  __HAL_TIM_DISABLE_IT(&htimbase, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
  // Enable TIM8 Update interrupt
  __HAL_TIM_ENABLE_IT(&htimbase, TIM_IT_UPDATE);
}

void vTaskDelayMs(uint32_t delay_milliseconds)
{
  uint32_t msToTicks;
  msToTicks = delay_milliseconds * configTICK_RATE_HZ / 1000;
  vTaskDelay(msToTicks);
}

void delay(uint32_t milliseconds)
{
  uint32_t msToTicks;
  msToTicks = milliseconds * configTICK_RATE_HZ / 1000;
  vTaskDelay(msToTicks);
}

// 获取 TIMx 的输入时钟频率（来源于RCC模块）
uint32_t tim_CK_INT_get(TIM_TypeDef *htim)
{
  uint32_t tmp;
  if ((htim == TIM1) || (htim == TIM8) || (htim == TIM9) || (htim == TIM10) || (htim == TIM11))
  { // APB2
    tmp = HAL_RCC_GetPCLK2Freq();
    if (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos] != 0)
    {
      tmp = tmp * 2;
    }
  }
  else
  { // APB1
    tmp = HAL_RCC_GetPCLK1Freq();
    if (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] != 0)
    {
      tmp = tmp * 2;
    }
  }

  return tmp;
}
