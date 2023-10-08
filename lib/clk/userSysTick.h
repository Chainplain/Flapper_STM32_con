/**

**/

#ifndef userSysTick_h
#define userSysTick_h

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

#define TIM_TIMEBASE TIM14
#define TIMBASE_IRQHandler TIM8_TRG_COM_TIM14_IRQHandler
#define TIMBASE_IRQn TIM8_TRG_COM_TIM14_IRQn
#define __HAL_RCC_TIM_TIMEBASE_CLK_ENABLE __HAL_RCC_TIM14_CLK_ENABLE
#define HAL_RCC_GetTimeBaseFreq HAL_RCC_GetPCLK1Freq

  void vTaskDelayMs(uint32_t delay_milliseconds);
  void delay(uint32_t milliseconds);
  uint32_t tim_CK_INT_get(TIM_TypeDef *htim);

  /**
 * @brief 此方法适用于case2: 采用 timestamp 优先级的 计数器中断，
 *        注意查看优先级设置userSystick.c/line42,
 *        此方法不能用于RTOS不能控制的中断，
 *        注意要考虑tick溢出的风险
 */
  inline uint64_t _micros(void) __attribute__((always_inline, unused));
  inline uint64_t _micros(void)
  { // 先让计数器停止
    // 采用双重判断时因为担心中时间uwTick更新发生在计算语句的时刻，
    // 采用此方法，可以得到稳定的时间
    // 但是此方法也会带来时间的延时（过多的计算）
    __IO uint64_t us_pre, us;
    __IO uint64_t tmp;
    tmp = uwTick * 1000;
    us_pre = tmp + TIM_TIMEBASE->CNT;
    tmp = uwTick * 1000;
    us = tmp + TIM_TIMEBASE->CNT;
    if ((us_pre <= us) && ((us - us_pre) < 500))
    {
    }
    else
    {
      tmp = uwTick * 1000;
      us = tmp + TIM_TIMEBASE->CNT;
    }
    return us;
  }

  /**
 * @brief us(微秒延时器)。
 *        影响准确性的因素有：中断，任务切换时间(configTICK_RATE_HZ)到时。
 *        因此该延时器用于小于任务切换时间的地方。
 * @param us the number of microseconds to pause (uint32_t)
 */
  static inline void delay_us(uint32_t) __attribute__((always_inline, unused));
  static inline void delay_us(uint32_t us)
  {
    __IO uint32_t currentTicks = TIM_TIMEBASE->CNT;
    // Number of ticks per millisecond
    const uint32_t tickPerMs = TIM_TIMEBASE->ARR;
    // Number of ticks to count
    const uint32_t nbTicks = ((us - ((us > 0) ? 1 : 0)) * tickPerMs) / 1000;
    // Number of elapsed ticks
    uint32_t elapsedTicks = 0;
    __IO uint32_t oldTicks = currentTicks;
    do
    {
      currentTicks = TIM_TIMEBASE->CNT;
      //elapsedTicks += (oldTicks < currentTicks) ? tickPerMs + oldTicks - currentTicks : oldTicks - currentTicks;
      elapsedTicks += (currentTicks >= oldTicks) ? (currentTicks - oldTicks) : (tickPerMs + currentTicks - oldTicks);
      oldTicks = currentTicks;
    } while (nbTicks > elapsedTicks);
  }

#ifdef __cplusplus
}
#endif

#endif
