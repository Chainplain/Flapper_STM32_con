
#include "userISR.h"
#include <main.h>

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
extern "C"
{
#ifdef userSysTick_h
	void TIMBASE_IRQHandler(void)
	{
		if ((TIM_TIMEBASE->SR & TIM_SR_UIF_Msk) == TIM_SR_UIF_Msk)
		{ // 代表 TIM14 发生中断，这里用作 非freeRTOS 的 systick
			// 替代 SysTick_Handler 中断
			// TIM_TIMEBASE->SR &= (~TIM_SR_UIF_Msk); // 0x00000001
			TIM_TIMEBASE->SR &= 0xFFFE; // 用此语句替换TIM_TIMEBASE->SR &= (~TIM_SR_UIF_Msk), 可以节省时间
			// HAL_IncTick();
			uwTick += uwTickFreq; // 用此语句替代HAL_IncTick();可以节省一些进出函数的时间
#ifdef watchdog_h
			IWDG_Refresh();
#endif
		}
		else
		{
			TIM_TIMEBASE->SR = 0;
		}
	}
#endif

#ifdef _ADC_H
	// void DMA2_Stream0_IRQHandler(void)
	void DMA_ADC_IRQHandler(void)
	{
		ADC_DMA_callback();
	}
#endif
}
