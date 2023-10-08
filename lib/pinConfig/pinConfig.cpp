/************************************************************
 * FileName:        pinConfig.cpp
 * Description:     pinConfig code
 *                  STM32 
 * Auther:          Jinsheng
 * CreateDate:      2021-06-01
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/

#include "pinConfig.h"

static void generalPinModeSet(GPIO_InitTypeDef *pInitStruct, uint16_t *gpio_pin, PinName pinName)
{
    *gpio_pin = get_Pin_Num(pinName);

    // enable GPIO port clock
    switch (get_Port_Num(pinName))
    {
    case 0:
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;
    case 1:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    case 2:
        __HAL_RCC_GPIOC_CLK_ENABLE();
        break;
    case 3:
        __HAL_RCC_GPIOD_CLK_ENABLE();
        break;
    case 4:
        __HAL_RCC_GPIOE_CLK_ENABLE();
        break;
    case 5:
        __HAL_RCC_GPIOF_CLK_ENABLE();
        break;
    case 6:
        __HAL_RCC_GPIOG_CLK_ENABLE();
        break;
    case 7:
        __HAL_RCC_GPIOH_CLK_ENABLE();
        break;
    case 8:
        __HAL_RCC_GPIOI_CLK_ENABLE();
        break;
    default:
        break;
    }

    pInitStruct->Pin = *gpio_pin;
    pInitStruct->Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    pInitStruct->Mode = GPIO_MODE_INPUT;
    pInitStruct->Pull = GPIO_NOPULL;
}

/**
 * @brief pin mode configuration
 * @param pinName PinName enum { PA0.., PB1.., PH2.. }
 * @param pinMode PinMode enum { INPUT, OUTPUT, INPUT_PULLDOWN, OUTPUT_OPEN_DRAIN,
 *                UART1, TIM1, IT_RISING, IT_FALLING, IT_RSING_FALLING,
 *                ANALOG,...}
 * @return 
 * */
void pinMode(PinName pinName, PinMode pinMode)
{
    uint16_t GPIO_Pin;
    GPIO_TypeDef *GPIOx = get_Port(pinName);
    GPIO_InitTypeDef GPIO_InitStruct;

    generalPinModeSet(&GPIO_InitStruct, &GPIO_Pin, pinName);

    switch (pinMode)
    {
    case ANALOG:
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG; //配置模式
        GPIO_InitStruct.Pull = GPIO_NOPULL;      //浮空输入
        break;
    case INPUT:
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        break;
    case INPUT_PULLUP:
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        break;
    case INPUT_PULLDOWN:
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        break;
    case OUTPUT:
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        break;
    case OUTPUT_OPEN_DRAIN:
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        break;
    case IT_RISING:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        break;
    case IT_FALLING:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        break;
    case IT_RISING_FALLING:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        break;
    case EVT_RISING:
        GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        break;
    case EVT_FALLING:
        GPIO_InitStruct.Mode = GPIO_MODE_EVT_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        break;
    case EVT_RISING_FALLING:
        GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        break;
    default:
        break;
    }

    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
 * @brief pin PWM mode configuration
 * @param pinName enum PinName. { PA0.., PB1.., PH2.. }
 * @param AFMode enum AFMode. PA8_TIM1_CH1_AF1, PB6_USART1_TX_AF7...
 * @param afOpenDrain bool type. Peripherial Position push-pull type
 *        [true, false], special for output pin
 * @param pullup bool type. Pin Position push-pull type,  
 *        PWM=false, UART=true
 * @return 
 * */
void pinModeAlternative(PinName pinName, AFMode afMode, bool afOpenDrain, bool pullup)
{
    uint16_t GPIO_Pin;
    GPIO_TypeDef *GPIOx = get_Port(pinName);
    GPIO_InitTypeDef GPIO_InitStruct;

    generalPinModeSet(&GPIO_InitStruct, &GPIO_Pin, pinName);

    GPIO_InitStruct.Alternate = afMode;

    if (pullup)
        GPIO_InitStruct.Pull = GPIO_PULLUP;
    else
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    if (afOpenDrain)
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    else
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
 * @brief set pin state in digital mode
 * @param pinName PinName enum { PA0.., PB1.., PH2.. }
 * @param PinState [0, 1] or [GPIO_PIN_RESET, GPIO_PIN_SET]
 * @return 
 * */
void digitalWrite(PinName pinName, uint8_t PinState)
{
    uint16_t GPIO_Pin = get_Pin_Num(pinName);
    GPIO_TypeDef *GPIOx = get_Port(pinName);

    if (PinState != GPIO_PIN_RESET)
    {
        GPIOx->BSRR = GPIO_Pin;
    }
    else
    {
        GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
    }
}

/**
 * @brief read pin state in digital mode
 * @param pinName PinName enum { PA0.., PB1.., PH2.. }
 * @return GPIO_PinState [0, 1] or [GPIO_PIN_RESET, GPIO_PIN_SET]
 * */
GPIO_PinState digitalRead(PinName pinName)
{
    uint16_t GPIO_Pin = get_Pin_Num(pinName);
    GPIO_TypeDef *GPIOx = get_Port(pinName);

    GPIO_PinState bitstatus;

    // Check the parameters
    assert_param(IS_GPIO_PIN(GPIO_Pin));

    if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
    {
        bitstatus = GPIO_PIN_SET;
    }
    else
    {
        bitstatus = GPIO_PIN_RESET;
    }
    return bitstatus;
}

/**
 * @brief toggle the pin state in digital mode
 * @param pinName PinName enum { PA0.., PB1.., PH2.. }
 * @return 
 * */
void digitalToggle(PinName pinName)
{
    uint16_t GPIO_Pin = get_Pin_Num(pinName);
    GPIO_TypeDef *GPIOx = get_Port(pinName);

    // Check the parameters
    assert_param(IS_GPIO_PIN(GPIO_Pin));

    if ((GPIOx->ODR & GPIO_Pin) == GPIO_Pin)
    {
        GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
    }
    else
    {
        GPIOx->BSRR = GPIO_Pin;
    }
}

// @brief Returns the IRQ number associated with a certain pin.
// Note that all GPIOs with the same pin number map to the same IRQn,
// no matter which port they belong to.
IRQn_Type get_irq_number(uint16_t pin)
{
    uint16_t pin_number = 0;
    pin >>= 1;
    while (pin)
    {
        pin >>= 1;
        pin_number++;
    }
    switch (pin_number)
    {
    case 0:
        return EXTI0_IRQn;
    case 1:
        return EXTI1_IRQn;
    case 2:
        return EXTI2_IRQn;
    case 3:
        return EXTI3_IRQn;
    case 4:
        return EXTI4_IRQn;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
        return EXTI9_5_IRQn;
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
        return EXTI15_10_IRQn;
    default:
        return (IRQn_Type)0; // impossible
    }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// gpio中断处理封装函数
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MAX_SUBSCRIPTIONS 16
struct subscription_t
{
    GPIO_TypeDef *GPIO_port;
    uint16_t GPIO_pin;
    void (*callback)(void *);
    void *ctx;
} subscriptions[MAX_SUBSCRIPTIONS] = {0};
size_t n_subscriptions = 0;

// Sets up the specified GPIO to trigger the specified callback
// on a rising edge of the GPIO.
// @param pull_up_down: one of GPIO_NOPULL, GPIO_PULLUP or GPIO_PULLDOWN
bool GPIO_subscribe(GPIO_TypeDef *GPIO_port, uint16_t GPIO_pin, uint32_t pull_up_down, void (*callback)(void *), void *ctx)
{
    // Register handler (or reuse existing registration)
    // TODO: make thread safe
    struct subscription_t *subscription = NULL;
    for (size_t i = 0; i < n_subscriptions; ++i)
    {
        if (subscriptions[i].GPIO_port == GPIO_port &&
            subscriptions[i].GPIO_pin == GPIO_pin)
            subscription = &subscriptions[i];
    }
    if (!subscription)
    {
        if (n_subscriptions >= MAX_SUBSCRIPTIONS)
            return false;
        subscription = &subscriptions[n_subscriptions++];
    }

    *subscription = (struct subscription_t){
        .GPIO_port = GPIO_port,
        .GPIO_pin = GPIO_pin,
        .callback = callback,
        .ctx = ctx};

    // Set up GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = pull_up_down;
    HAL_GPIO_Init(GPIO_port, &GPIO_InitStruct);

    // Clear any previous triggers
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_pin);
    // Enable interrupt
    HAL_NVIC_SetPriority(get_irq_number(GPIO_pin), 0, 0);
    HAL_NVIC_EnableIRQ(get_irq_number(GPIO_pin));
    return true;
}

void GPIO_unsubscribe(GPIO_TypeDef *GPIO_port, uint16_t GPIO_pin)
{
    bool is_pin_in_use = false;
    for (size_t i = 0; i < n_subscriptions; ++i)
    {
        if (subscriptions[i].GPIO_port == GPIO_port &&
            subscriptions[i].GPIO_pin == GPIO_pin)
        {
            subscriptions[i].callback = NULL;
            subscriptions[i].ctx = NULL;
        }
        else if (subscriptions[i].GPIO_pin == GPIO_pin)
        {
            is_pin_in_use = true;
        }
    }
    if (!is_pin_in_use)
        HAL_NVIC_DisableIRQ(get_irq_number(GPIO_pin));
}

//Dispatch processing of external interrupts based on source
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin)
{
    for (size_t i = 0; i < n_subscriptions; ++i)
    {
        if (subscriptions[i].GPIO_pin == GPIO_pin) // TODO: check for port
            if (subscriptions[i].callback)
                subscriptions[i].callback(subscriptions[i].ctx);
    }
}
