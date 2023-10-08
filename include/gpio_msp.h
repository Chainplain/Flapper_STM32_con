/**

**/

#ifndef _GPIO_MSP_H
#define _GPIO_MSP_H

#include <stm32f4xx_hal.h>
#include <pinConfig.h>

// ADC
#define VI_PIN       GPIO_PIN_0
#define VI_PORT      GPIOA
#define ADC1_PIN     GPIO_PIN_1
#define ADC1_PORT    GPIOA
#define ADC2_PIN     GPIO_PIN_2
#define ADC2_PORT    GPIOA
#define ADC3_PIN     GPIO_PIN_3
#define ADC3_PORT    GPIOA
#define ADC4_PIN     GPIO_PIN_6
#define ADC4_PORT    GPIOA
#define ADC5_PIN     GPIO_PIN_7
#define ADC5_PORT    GPIOA
#define ADC6_PIN     GPIO_PIN_0
#define ADC6_PORT    GPIOB
#define ADC7_PIN     GPIO_PIN_1
#define ADC7_PORT    GPIOB

// DAC
// 忽略即可，保留默认值

// PWM
#define PWM1_PIN		GPIO_PIN_6 // TIM3/TIM8
#define PWM1_PORT		GPIOC
#define PWM2_PIN		GPIO_PIN_7
#define PWM2_PORT		GPIOC
#define PWM3_PIN		GPIO_PIN_8
#define PWM3_PORT		GPIOC
#define PWM4_PIN		GPIO_PIN_9
#define PWM4_PORT		GPIOC
#define PWM5_PIN		GPIO_PIN_8 // TIM1
#define PWM5_PORT		GPIOA
#define PWM6_PIN		GPIO_PIN_9
#define PWM6_PORT		GPIOA
#define PWM7_PIN		GPIO_PIN_10
#define PWM7_PORT		GPIOA

// LED
#define LED1_PIN          GPIO_PIN_11
#define LED1_PORT         GPIOA
#define LED2_PIN          GPIO_PIN_12
#define LED2_PORT         GPIOA
#define LED3_PIN          GPIO_PIN_10
#define LED3_PORT         GPIOC
#define LED4_PIN          GPIO_PIN_3
#define LED4_PORT         GPIOB
#define LED5_PIN          GPIO_PIN_4
#define LED5_PORT         GPIOB
#define LED6_PIN          GPIO_PIN_5
#define LED6_PORT         GPIOB
#define LED7_PIN          GPIO_PIN_6
#define LED7_PORT         GPIOB
#define LED8_PIN          GPIO_PIN_7
#define LED8_PORT         GPIOB
#define LED9_PIN          GPIO_PIN_8
#define LED9_PORT         GPIOB
#define LED10_PIN         GPIO_PIN_9
#define LED10_PORT        GPIOB

//USART
#define TX3_PIN		    GPIO_PIN_10 //USTRT3 -> ESP8266
#define TX3_PORT	    GPIOB
#define RX3_PIN		    GPIO_PIN_11
#define RX3_PORT	    GPIOB

#define TX4_PIN		    GPIO_PIN_10 //USTRT3 -> IMU
#define TX4_PORT		GPIOC
#define RX4_PIN		    GPIO_PIN_11
#define RX4_PORT		GPIOC

#define TX5_PIN		    GPIO_PIN_12 //USTRT5 -> for user communication
#define TX5_PORT		GPIOC
#define RX5_PIN		    GPIO_PIN_2
#define RX5_PORT		GPIOD

//SPI
#define SPINSS_PIN		    GPIO_PIN_12
#define SPINSS_PORT		    GPIOB
#define SPISCK_PIN		    GPIO_PIN_13
#define SPISCK_PORT		GPIOB
#define SPIMISO_PIN		    GPIO_PIN_14
#define SPIMISO_PORT		GPIOB
#define SPIMOSI_PIN		    GPIO_PIN_15
#define SPIMOSI_PORT		GPIOB

//EN
#define REST_PIN         GPIO_PIN_4
#define REST_PORT        GPIOC
#define GPIO0_PIN         GPIO_PIN_5
#define GPIO0_PORT        GPIOC

//GPIO
#define D0_PIN          GPIO_PIN_0
#define D0_PORT         GPIOC
#define D1_PIN          GPIO_PIN_1
#define D1_PORT         GPIOC
#define D2_PIN          GPIO_PIN_2
#define D2_PORT         GPIOC
#define D3_PIN          GPIO_PIN_3
#define D3_PORT         GPIOC

#define PH1_PIN         GPIO_PIN_1
#define PH1_PORT        GPIOH
#define PC13_PIN          GPIO_PIN_13
#define PC13_PORT         GPIOC
#define PC14_PIN          GPIO_PIN_14
#define PC14_PORT         GPIOC
#define PC15_PIN          GPIO_PIN_15
#define PC15_PORT         GPIOC

void GPIO_PINs_Init(void);
void GPIO_output_IO_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void GPIO_input_IO_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void GPIO_uart_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx);
void GPIO_analog_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void GPIO_pwm_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx);
void GPIO_MCO1_init(void);
void GPIO_SPI_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx);
void GPIO_I2C_init(GPIO_TypeDef *scl_gpio, uint32_t scl_pin, GPIO_TypeDef *sda_gpio, uint32_t sda_pin, uint8_t AFx);
void GPIO_CAN_init(GPIO_TypeDef *tx_gpio, uint32_t tx_pin, GPIO_TypeDef *rx_gpio, uint32_t rx_pin, uint8_t AFx);
void GPIO_IR_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint32_t mode);

#endif
