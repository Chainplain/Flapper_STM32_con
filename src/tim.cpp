/************************************************************
 * FileName:        tim.cpp
 * Description:     定时器配置[除了timebase定时器TIM14以外]
 *                  STM32
 * Auther:          Jinsheng
 * CreateDate:      2021-09-05
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/

#include <main.h>
#include "tim.h"
#include "stm32f4xx_hal_tim_ex.h"
#include "stm32f4xx_hal_tim.h"

#define PWM_FREQUENCY 500000 // 开关频率500kHz

static void pwm_init(TIM_HandleTypeDef *htim, TIM_TypeDef *TIM);
static void refbase_tim_init(TIM_TypeDef *TIM, uint32_t period, uint32_t prescaler);
static void sync_timers(TIM_TypeDef *TIM_Master, TIM_TypeDef *TIM_Slave, uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset);

void tim_init(void)
{
    TIM_HandleTypeDef htim1, htim8;
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    pwm_init(&htim1, TIM1);
    pwm_init(&htim8, TIM8);

    // add pin configurations here
    // done in gpio_msp.cpp

    // enable
    // register: cr1(cen bit), ccer(cc1e, cc1ne, ...bits),  bdtr(moe bit)
    // see HAL_TIM_PWM_Start, HAL_TIMEx_PWMN_Start
    htim1.Instance->CCER |= 0x0111;
    htim1.Instance->CCMR1 |= 0x0808; // ocref1, ocref2
    htim1.Instance->CCMR2 |= 0x0808; // ocref3, ocref4

    htim8.Instance->CCER |= 0x1111;
    htim8.Instance->CCMR1 |= 0x0808; // ocref1, ocref2
    htim8.Instance->CCMR2 |= 0x0808; // ocref3, ocref4

    // 关闭pin脚的PWM输出
    htim1.Instance->CCR1 = htim1.Init.Period;
    htim1.Instance->CCR2 = htim1.Init.Period;
    htim1.Instance->CCR3 = htim1.Init.Period;
    htim8.Instance->CCR1 = htim8.Init.Period;
    htim8.Instance->CCR2 = htim8.Init.Period;
    htim8.Instance->CCR3 = htim8.Init.Period;
    htim8.Instance->CCR4 = htim8.Init.Period;

    //__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1); // 关闭定时器的pin脚输出
    // htim1.Instance->CR1 |= TIM_CR1_CEN;            // 启动定时器，这里关闭，将在sync_timers函数里开启

    //    refbase_tim_init(TIM8, htim1.Init.Period + 1); // TIM1和TIM8是同一个总线时钟:APB2

    // TRGO for ADC
    uint32_t tmpcr2 = htim1.Instance->CR2;
    tmpcr2 &= ~TIM_CR2_MMS;
    tmpcr2 |= TIM_TRGO_UPDATE;    // ; TIM_TRGO_OC4REF
    htim1.Instance->CR2 = tmpcr2; // set update event as TRGO

    __HAL_TIM_MOE_DISABLE(&htim1); // 定时器的pin脚输出
    __HAL_TIM_MOE_DISABLE(&htim8); // 定时器的pin脚输出

    refbase_tim_init(TIM8, htim1.Init.Period, htim1.Init.Prescaler); // TIM1周期和TIM8一样，一个是center align, 一个是couterup, TIM1和TIM8是同一个总线时钟:APB2
    sync_timers(TIM1, TIM8, TIM_CLOCKSOURCE_ITR0, 0);                // tim8 TIM_CLOCKSOURCE_ITR0 = tim1

    __HAL_TIM_MOE_ENABLE(&htim1); // 使能定时器的pin脚输出
    __HAL_TIM_MOE_ENABLE(&htim8); // 使能定时器的pin脚输出

    // 设置中断
    // TIM8->DIER |= TIM_DIER_UIE; // update interrupt enable
    // TIM8->SR = 0;               // clear all flags
    // HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    // HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    // TIM1->DIER |= TIM_DIER_UIE; // update interrupt enable
    // TIM1->SR = 0; // clear all flags
    // HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    // HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

static void pwm_init(TIM_HandleTypeDef *htim, TIM_TypeDef *TIM)
{
    TIM_MasterConfigTypeDef sMasterConfig = {};
    TIM_OC_InitTypeDef sConfigOC = {};

    uint32_t uwTimclock = tim_CK_INT_get(TIM);                              // get TIM clock cnt frequency
    uint32_t prescaler = 1;                                                 // clock prescaler
    uint32_t period = (uint32_t)(uwTimclock / prescaler / (PWM_FREQUENCY)); // calc period

    // 配置三角波PWM格式
    htim->Instance = TIM;
    htim->Init.Prescaler = (prescaler - 1);
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = period - 1;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(htim);

    // 从模式关闭 [独立的PWM], 选择信号给TRGO输出
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig);

    // PWM1模式(out=1 when cnt<ccr), idle和reset状态下输出为0
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_4);

    // auto-reload
    TIM->CR1 |= TIM_CR1_ARPE; // enable period auto-reload buffer
    // TIM->CCMR1 |= TIM_CCMR1_OC1PE; // enable compare auto-reload buffer
    // TIM->CCMR1 |= TIM_CCMR1_OC2PE; // enable compare auto-reload buffer
    // TIM->CCMR2 |= TIM_CCMR2_OC3PE; // enable compare auto-reload buffer

    // add pin configurations here
    // ...

    // enable
    // register: cr1(cen bit), ccer(cc1e, cc1ne, ...bits),  bdtr(moe bit)
    // see HAL_TIM_PWM_Start, HAL_TIMEx_PWMN_Start
}

/** 定时器同步
 * 注意要查看 timer 的总线时钟是哪一个
 * TIM8将被用来测量中断电流环运算的时间长度
 * */
static void refbase_tim_init(TIM_TypeDef *TIM, uint32_t period, uint32_t prescaler)
{
    TIM_HandleTypeDef htim;
    htim.Instance = TIM;
    htim.Init.Prescaler = prescaler;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = period;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim.Init.RepetitionCounter = 0;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim);
}

/**
 * 同步的方法：
 *  -1 保存 master CR2, slave SMCR的值
 *  0 先关闭pin的输出，对于3相桥来说，关闭BDTR寄存器的MOE即可
 *  1 关闭CR1寄存器的CEN, 准备开始
 *  2 master CR2寄存器的MMS选择master的TRGO信号源，用于触发slave timer，选择CEN_ENABLE作为触发信号
 *  3 slave SMCR寄存器的TS选择触发源来自哪里，选择Internal Trigger x
 *  4 slave SMCR寄存器的SMS选择触发方式，选择Gated Mode门触发方式，配合TS选择的TRGI
 *  5 CR1寄存器的CMS和DIR设置和确定和设置方向
 *  6 master CNT寄存器设置偏置值，slave CNT寄存器清零
 *  7 master CR1寄存器的CEN置位，开始计数，同时会触发slave CEN=1，slave也开始计数
 *  8 恢复master CR2和slave SMCR的值
 *  9 开通pin的输出
 */
static void sync_timers(TIM_TypeDef *TIM_Master, TIM_TypeDef *TIM_Slave, uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset)
{
    // Store intial timer configs
    uint16_t CR2_store = TIM_Master->CR2;
    uint16_t SMCR_store = TIM_Slave->SMCR;
    // Disable both timer counters
    TIM_Master->CR1 &= ~TIM_CR1_CEN;
    TIM_Slave->CR1 &= ~TIM_CR1_CEN;
    // Set first timer to send TRGO on counter enable
    TIM_Master->CR2 &= ~TIM_CR2_MMS;
    TIM_Master->CR2 |= TIM_TRGO_ENABLE;
    // Set Trigger Source of second timer to the TRGO of the first timer
    TIM_Slave->SMCR &= ~TIM_SMCR_TS;
    TIM_Slave->SMCR |= TIM_CLOCKSOURCE_ITRx;
    // Set 2nd timer to start on trigger
    TIM_Slave->SMCR &= ~TIM_SMCR_SMS;
    TIM_Slave->SMCR |= TIM_SLAVEMODE_TRIGGER;
    // Dir bit is read only in center aligned mode, so we clear the mode for now
    uint16_t CMS_store_a = TIM_Master->CR1 & TIM_CR1_CMS;
    uint16_t CMS_store_b = TIM_Slave->CR1 & TIM_CR1_CMS;
    TIM_Master->CR1 &= ~TIM_CR1_CMS;
    TIM_Slave->CR1 &= ~TIM_CR1_CMS;
    // Set both timers to up-counting state
    TIM_Master->CR1 &= ~TIM_CR1_DIR;
    TIM_Slave->CR1 &= ~TIM_CR1_DIR;
    // Restore center aligned mode
    TIM_Master->CR1 |= CMS_store_a;
    TIM_Slave->CR1 |= CMS_store_b;
    // set counter offset
    TIM_Master->CNT = 0;
    TIM_Slave->CNT = count_offset;
    // Start Timer a
    TIM_Master->CR1 |= (TIM_CR1_CEN);
    // Restore timer configs
    TIM_Master->CR2 = CR2_store;
    TIM_Slave->SMCR = SMCR_store;
}

//舵机PWM控制--高电平一般为0.5ms-2.5ms范围内的角度控制脉冲，总间隔为2ms，以180°伺服为例；
//0.5ms---0°
//1.0ms---45°
//1.5ms---90°
//2.0ms---135°
//2.5ms---180°
void PWM_duty_set(uint8_t index, float duty)
{   
    TIM_TypeDef *timer = ((index < 4) ? TIM8 : TIM1);

    if (duty < 0.0f)
        duty = 0.0f;
    if (duty > 1.0f)
        duty = 1.0f;

    uint16_t ccr = (uint16_t)(duty * (timer->ARR + 1));

    switch (index)
    {
    case 0:
        timer->CCR1 = ccr;
        break;
    case 1:
        timer->CCR2 = ccr;
        break;
    case 2:
        timer->CCR3 = ccr;
        break;
    case 3:
        timer->CCR4 = ccr;
        break;
    case 4:
        timer->CCR1 = ccr;
        break;
    case 5:
        timer->CCR2 = ccr;
        break;
        case 6:
        timer->CCR3 = ccr;
        break;
    }
}

float PWM_duty_get(uint8_t index)
{
    float duty = 0.0f;
    float period = TIM1->ARR + 1;

    switch (index)
    {
    case 0:
        duty = ((float)TIM1->CCR1) / period;
        break;
    case 1:
        duty = ((float)TIM1->CCR2) / period;
        break;
    case 2:
        duty = ((float)TIM1->CCR3) / period;
        break;
    default:
        break;
    }

    return duty;
}
