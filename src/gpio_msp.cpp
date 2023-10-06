/**
	this is for Piezoelectrical control

	include 4 pwm output  --> tim1
					2 gpio output --> for enable control

	by jinsheng 2019-12-27
**/

#include "gpio_msp.h"

GPIO_InitTypeDef GPIO_InitStruct; // 定义初始化结构体

void GPIO_PINs_Init(void) // 初始化函数
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	// ADC
	GPIO_analog_init(VI_PORT, VI_PIN);
	GPIO_analog_init(ADC1_PORT, ADC1_PIN);
	GPIO_analog_init(ADC2_PORT, ADC2_PIN);
	GPIO_analog_init(ADC3_PORT, ADC3_PIN);
	GPIO_analog_init(ADC4_PORT, ADC4_PIN);
	GPIO_analog_init(ADC5_PORT, ADC5_PIN);
	GPIO_analog_init(ADC6_PORT, ADC6_PIN);
	GPIO_analog_init(ADC7_PORT, ADC7_PIN);

	// PWM
	GPIO_pwm_init(PWM1_PORT, PWM1_PIN, GPIO_AF3_TIM8);
	GPIO_pwm_init(PWM2_PORT, PWM2_PIN, GPIO_AF3_TIM8);
	GPIO_pwm_init(PWM3_PORT, PWM3_PIN, GPIO_AF3_TIM8);
	GPIO_pwm_init(PWM4_PORT, PWM4_PIN, GPIO_AF3_TIM8);
	GPIO_pwm_init(PWM5_PORT, PWM5_PIN, GPIO_AF1_TIM1);
	GPIO_pwm_init(PWM6_PORT, PWM6_PIN, GPIO_AF1_TIM1);
	GPIO_pwm_init(PWM7_PORT, PWM7_PIN, GPIO_AF1_TIM1);

	// led pin
	HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
	GPIO_output_IO_init(LED1_PORT, LED1_PIN);

	//HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);
	//GPIO_output_IO_init(LED2_PORT, LED2_PIN);

	// uart
	GPIO_uart_init(TX5_PORT, TX5_PIN, GPIO_AF8_UART5);
	GPIO_uart_init(RX5_PORT, RX5_PIN, GPIO_AF8_UART5);

	// SPI
	GPIO_SPI_init(SPINSS_PORT, SPINSS_PIN, GPIO_AF1_TIM1);
	GPIO_SPI_init(SPISCK_PORT, SPISCK_PIN, GPIO_AF1_TIM1);
	GPIO_SPI_init(SPIMISO_PORT, SPIMISO_PIN, GPIO_AF1_TIM1);
	GPIO_SPI_init(SPIMOSI_PORT, SPIMOSI_PIN, GPIO_AF1_TIM1);

	// GPIO
	//GPIO_input_IO_init(D0_PORT, D0_PIN);
	//GPIO_input_IO_init(D1_PORT, D1_PIN);
	//GPIO_input_IO_init(D2_PORT, D2_PIN);
	//GPIO_input_IO_init(D3_PORT, D3_PIN);
}

void GPIO_analog_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG; // 配置模式
	GPIO_InitStruct.Pull = GPIO_NOPULL;		 // 浮空输入

	GPIO_InitStruct.Pin = GPIO_Pin;			// 配置GPIO_Pin  IO口
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct); // 初始化GPIOx的参数为以上结构体
}

void GPIO_output_IO_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = GPIO_Pin;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_input_IO_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = GPIO_Pin;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_uart_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx)
{
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	// note
	// GPIO_AF7_USART1 = 0x07
	// GPIO_AF8_LPUART1 = 0x08
	GPIO_InitStruct.Alternate = AFx;

	GPIO_InitStruct.Pin = GPIO_Pin;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_pwm_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx)
{
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	// note
	// GPIO_AF1_TIM1 = 0x01
	// GPIO_AF1_TIM2 = 0x01
	GPIO_InitStruct.Alternate = AFx;

	GPIO_InitStruct.Pin = GPIO_Pin;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_MCO1_init(void)
{
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_MCO;

	GPIO_InitStruct.Pin = GPIO_PIN_8;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void GPIO_SPI_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx)
{
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP; // required for disconnect detection on SPI encoders
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = AFx;
	GPIO_InitStruct.Pin = GPIO_Pin;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_I2C_init(GPIO_TypeDef *scl_gpio, uint32_t scl_pin, GPIO_TypeDef *sda_gpio, uint32_t sda_pin, uint8_t AFx)
{
	// I2C TX GPIO pin configuration
	GPIO_InitStruct.Pin = scl_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = AFx;

	HAL_GPIO_Init(scl_gpio, &GPIO_InitStruct);

	// I2C RX GPIO pin configuration
	GPIO_InitStruct.Pin = sda_pin;
	GPIO_InitStruct.Alternate = AFx;

	HAL_GPIO_Init(sda_gpio, &GPIO_InitStruct);
}

void GPIO_CAN_init(GPIO_TypeDef *tx_gpio, uint32_t tx_pin, GPIO_TypeDef *rx_gpio, uint32_t rx_pin, uint8_t AFx)
{
	// CAN1 TX GPIO pin configuration
	GPIO_InitStruct.Pin = tx_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = AFx;

	HAL_GPIO_Init(tx_gpio, &GPIO_InitStruct);

	// CAN1 RX GPIO pin configuration
	GPIO_InitStruct.Pin = rx_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = AFx;

	HAL_GPIO_Init(rx_gpio, &GPIO_InitStruct);
}

void GPIO_IR_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint32_t mode)
{
	GPIO_InitStruct.Mode = mode;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = GPIO_Pin;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
