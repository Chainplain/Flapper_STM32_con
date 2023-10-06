#pragma once

#ifndef pinTypes_h
#define pinTypes_h

#ifdef __cplusplus
extern "C"
{
#endif

#include <stm32f4xx_hal.h>

extern GPIO_TypeDef *GPIOPort[];

typedef enum {
  FirstPort = 0x00,
  PortA = FirstPort,
  PortB,
#if defined GPIOC_BASE
  PortC,
#endif
#if defined GPIOD_BASE
  PortD,
#endif
#if defined GPIOE_BASE
  PortE,
#endif
#if defined GPIOF_BASE
  PortF,
#endif
#if defined GPIOG_BASE
  PortG,
#endif
#if defined GPIOH_BASE
  PortH,
#endif
#if defined GPIOI_BASE
  PortI,
#endif
#if defined GPIOJ_BASE
  PortJ,
#endif
#if defined GPIOK_BASE
  PortK,
#endif
#if defined GPIOZ_BASE
  PortZ,
#endif
  PortEND,
  LastPort = PortEND - 1
} PortName;

#define MAX_NB_PORT (LastPort-FirstPort+1)

/* Return GPIO base address */
#define get_GPIO_Port(p) ((p < MAX_NB_PORT) ? GPIOPort[p] : (GPIO_TypeDef *)NULL)
/* Enable GPIO clock and return GPIO base address */
GPIO_TypeDef *set_GPIO_Port_Clock(uint32_t port_idx);

typedef enum {
  // Not connected
  NC = 0xFFFFFFFF,

  // Pin name definition
  PA0  = (PortA << 4) + 0x00,
  PA1  = (PortA << 4) + 0x01,
  PA2  = (PortA << 4) + 0x02,
  PA3  = (PortA << 4) + 0x03,
  PA4  = (PortA << 4) + 0x04,
  PA5  = (PortA << 4) + 0x05,
  PA6  = (PortA << 4) + 0x06,
  PA7  = (PortA << 4) + 0x07,
  PA8  = (PortA << 4) + 0x08,
  PA9  = (PortA << 4) + 0x09,
  PA10 = (PortA << 4) + 0x0A,
  PA11 = (PortA << 4) + 0x0B,
  PA12 = (PortA << 4) + 0x0C,
  PA13 = (PortA << 4) + 0x0D,
  PA14 = (PortA << 4) + 0x0E,
  PA15 = (PortA << 4) + 0x0F,

  PB0  = (PortB << 4) + 0x00,
  PB1  = (PortB << 4) + 0x01,
  PB2  = (PortB << 4) + 0x02,
  PB3  = (PortB << 4) + 0x03,
  PB4  = (PortB << 4) + 0x04,
  PB5  = (PortB << 4) + 0x05,
  PB6  = (PortB << 4) + 0x06,
  PB7  = (PortB << 4) + 0x07,
  PB8  = (PortB << 4) + 0x08,
  PB9  = (PortB << 4) + 0x09,
  PB10 = (PortB << 4) + 0x0A,
  PB11 = (PortB << 4) + 0x0B,
  PB12 = (PortB << 4) + 0x0C,
  PB13 = (PortB << 4) + 0x0D,
  PB14 = (PortB << 4) + 0x0E,
  PB15 = (PortB << 4) + 0x0F,
#if defined GPIOC_BASE
  PC0  = (PortC << 4) + 0x00,
  PC1  = (PortC << 4) + 0x01,
  PC2  = (PortC << 4) + 0x02,
  PC3  = (PortC << 4) + 0x03,
  PC4  = (PortC << 4) + 0x04,
  PC5  = (PortC << 4) + 0x05,
  PC6  = (PortC << 4) + 0x06,
  PC7  = (PortC << 4) + 0x07,
  PC8  = (PortC << 4) + 0x08,
  PC9  = (PortC << 4) + 0x09,
  PC10 = (PortC << 4) + 0x0A,
  PC11 = (PortC << 4) + 0x0B,
  PC12 = (PortC << 4) + 0x0C,
  PC13 = (PortC << 4) + 0x0D,
  PC14 = (PortC << 4) + 0x0E,
  PC15 = (PortC << 4) + 0x0F,
#endif
#if defined GPIOD_BASE
  PD0  = (PortD << 4) + 0x00,
  PD1  = (PortD << 4) + 0x01,
  PD2  = (PortD << 4) + 0x02,
  PD3  = (PortD << 4) + 0x03,
  PD4  = (PortD << 4) + 0x04,
  PD5  = (PortD << 4) + 0x05,
  PD6  = (PortD << 4) + 0x06,
  PD7  = (PortD << 4) + 0x07,
  PD8  = (PortD << 4) + 0x08,
  PD9  = (PortD << 4) + 0x09,
  PD10 = (PortD << 4) + 0x0A,
  PD11 = (PortD << 4) + 0x0B,
  PD12 = (PortD << 4) + 0x0C,
  PD13 = (PortD << 4) + 0x0D,
  PD14 = (PortD << 4) + 0x0E,
  PD15 = (PortD << 4) + 0x0F,
#endif
#if defined GPIOE_BASE
  PE0  = (PortE << 4) + 0x00,
  PE1  = (PortE << 4) + 0x01,
  PE2  = (PortE << 4) + 0x02,
  PE3  = (PortE << 4) + 0x03,
  PE4  = (PortE << 4) + 0x04,
  PE5  = (PortE << 4) + 0x05,
  PE6  = (PortE << 4) + 0x06,
  PE7  = (PortE << 4) + 0x07,
  PE8  = (PortE << 4) + 0x08,
  PE9  = (PortE << 4) + 0x09,
  PE10 = (PortE << 4) + 0x0A,
  PE11 = (PortE << 4) + 0x0B,
  PE12 = (PortE << 4) + 0x0C,
  PE13 = (PortE << 4) + 0x0D,
  PE14 = (PortE << 4) + 0x0E,
  PE15 = (PortE << 4) + 0x0F,
#endif
#if defined GPIOF_BASE
  PF0  = (PortF << 4) + 0x00,
  PF1  = (PortF << 4) + 0x01,
  PF2  = (PortF << 4) + 0x02,
  PF3  = (PortF << 4) + 0x03,
  PF4  = (PortF << 4) + 0x04,
  PF5  = (PortF << 4) + 0x05,
  PF6  = (PortF << 4) + 0x06,
  PF7  = (PortF << 4) + 0x07,
  PF8  = (PortF << 4) + 0x08,
  PF9  = (PortF << 4) + 0x09,
  PF10 = (PortF << 4) + 0x0A,
  PF11 = (PortF << 4) + 0x0B,
  PF12 = (PortF << 4) + 0x0C,
  PF13 = (PortF << 4) + 0x0D,
  PF14 = (PortF << 4) + 0x0E,
  PF15 = (PortF << 4) + 0x0F,
#endif
#if defined GPIOG_BASE
  PG0  = (PortG << 4) + 0x00,
  PG1  = (PortG << 4) + 0x01,
  PG2  = (PortG << 4) + 0x02,
  PG3  = (PortG << 4) + 0x03,
  PG4  = (PortG << 4) + 0x04,
  PG5  = (PortG << 4) + 0x05,
  PG6  = (PortG << 4) + 0x06,
  PG7  = (PortG << 4) + 0x07,
  PG8  = (PortG << 4) + 0x08,
  PG9  = (PortG << 4) + 0x09,
  PG10 = (PortG << 4) + 0x0A,
  PG11 = (PortG << 4) + 0x0B,
  PG12 = (PortG << 4) + 0x0C,
  PG13 = (PortG << 4) + 0x0D,
  PG14 = (PortG << 4) + 0x0E,
  PG15 = (PortG << 4) + 0x0F,
#endif
#if defined GPIOH_BASE
  PH0  = (PortH << 4) + 0x00,
  PH1  = (PortH << 4) + 0x01,
  PH2  = (PortH << 4) + 0x02,
  PH3  = (PortH << 4) + 0x03,
  PH4  = (PortH << 4) + 0x04,
  PH5  = (PortH << 4) + 0x05,
  PH6  = (PortH << 4) + 0x06,
  PH7  = (PortH << 4) + 0x07,
  PH8  = (PortH << 4) + 0x08,
  PH9  = (PortH << 4) + 0x09,
  PH10 = (PortH << 4) + 0x0A,
  PH11 = (PortH << 4) + 0x0B,
  PH12 = (PortH << 4) + 0x0C,
  PH13 = (PortH << 4) + 0x0D,
  PH14 = (PortH << 4) + 0x0E,
  PH15 = (PortH << 4) + 0x0F,
#endif
#if defined GPIOI_BASE
  PI0  = (PortI << 4) + 0x00,
  PI1  = (PortI << 4) + 0x01,
  PI2  = (PortI << 4) + 0x02,
  PI3  = (PortI << 4) + 0x03,
  PI4  = (PortI << 4) + 0x04,
  PI5  = (PortI << 4) + 0x05,
  PI6  = (PortI << 4) + 0x06,
  PI7  = (PortI << 4) + 0x07,
  PI8  = (PortI << 4) + 0x08,
  PI9  = (PortI << 4) + 0x09,
  PI10 = (PortI << 4) + 0x0A,
  PI11 = (PortI << 4) + 0x0B,
  PI12 = (PortI << 4) + 0x0C,
  PI13 = (PortI << 4) + 0x0D,
  PI14 = (PortI << 4) + 0x0E,
  PI15 = (PortI << 4) + 0x0F,
#endif
}PinName;

// pinName = PinName enum { PA0.., PB1.., PH2.. }
// return GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2
#define get_Pin_Num(pinName) (uint16_t)(1U << (pinName & 0x0F))

// pinName = PinName enum { PA0.., PB1.., PH2.. }
// return PORTA, PORTB, PORTH
#define get_Port_Num(pinName)  ((pinName & 0xF0) >> 4)

// pinName = PinName enum { PA0.., PB1.., PH2.. }
// return GPIO_TypeDef* : GPIOA, GPIOB, GPIOH
#define get_Port(pinName) get_GPIO_Port(get_Port_Num(pinName))

typedef enum {
  INPUT = 0x0,
  INPUT_FLOATING = 0x0,
  INPUT_PULLUP = 0x1,  
  INPUT_PULLDOWN = 0x2,
  OUTPUT = 0x3,
  OUTPUT_OPEN_DRAIN = 0x4,  
  ANALOG = 0x5,
  IT_RISING = 0x6,
  IT_FALLING = 0X7,
  IT_RISING_FALLING = 0x8,
  EVT_RISING = 0x9,
  EVT_FALLING = 0xA,
  EVT_RISING_FALLING = 0xB,
}PinMode;

typedef enum {
  #pragma region Timer Alternative Function Selection
  // TIM1
  PA6_TIM1_BKIN_AF1 = 0x01,
  PA7_TIM1_CH1N_AF1 = 0x01,
  PA8_TIM1_CH1_AF1 = 0x01,
  PA9_TIM1_CH2_AF1 = 0x01,
  PA10_TIM1_CH3_AF1 = 0x01,
  PA11_TIM1_CH4_AF1 = 0x01,
  PA12_TIM1_ETR_AF1 = 0x01,
  PB0_TIM1_CH2N_AF1 = 0x01,
  PB1_TIM1_CH3N_AF1 = 0x01,
  PB12_TIM1_BKIN_AF1 = 0x01,
  PB13_TIM1_CH1N_AF1 = 0x01,
  PB14_TIM1_CH2N_AF1 = 0x01,
  PB15_TIM1_CH3N_AF1 = 0x01,

  // TIM2
  PA0_TIM2_CH1_AF1 = 0x01,
  PA1_TIM2_CH2_AF1 = 0x01,
  PA2_TIM2_CH3_AF1 = 0x01,
  PA3_TIM2_CH4_AF1 = 0x01,
  PA5_TIM2_CH1_AF1 = 0x01,
  PA15_TIM2_CH1_AF1 = 0x01,
  PB3_TIM2_CH2_AF1 = 0x01,
  PB10_TIM2_CH3_AF1 = 0x01,
  PB11_TIM2_CH4_AF1 = 0x01,

  // TIM3
  PA6_TIM3_CH1_AF2 = 0x02,
  PA7_TIM3_CH2_AF2 = 0x02,
  PB0_TIM3_CH3_AF2 = 0x02,
  PB1_TIM3_CH4_AF2 = 0x02,
  PB4_TIM3_CH1_AF2 = 0x02,
  PB5_TIM3_CH2_AF2 = 0x02,
  PC6_TIM3_CH1_AF2 = 0x02,
  PC7_TIM3_CH2_AF2 = 0x02,
  PC8_TIM3_CH3_AF2 = 0x02,
  PC9_TIM3_CH4_AF2 = 0x02,
  PD2_TIM3_ETR_AF2 = 0x02,

  // TIM4
  PB6_TIM4_CH1_AF2 = 0x02,
  PB7_TIM4_CH2_AF2 = 0x02,
  PB8_TIM4_CH3_AF2 = 0x02,
  PB9_TIM4_CH4_AF2 = 0x02,

  // TIM5
  PA0_TIM5_CH1_AF2 = 0x02,
  PA1_TIM5_CH2_AF2 = 0x02,
  PA2_TIM5_CH3_AF2 = 0x02,
  PA3_TIM5_CH4_AF2 = 0x02,

  // TIM8
  PA0_TIM8_CH2N_AF3 = 0x03,
  PA1_TIM8_CH3N_AF3 = 0x03,
  PB14_TIM8_CH2N_AF3 = 0x03,
  PB15_TIM8_CH3N_AF3 = 0x03,
  PC6_TIM8_CH1_AF3 = 0x03,
  PC7_TIM8_CH2_AF3 = 0x03,
  PC8_TIM8_CH3_AF3 = 0x03,
  PC9_TIM8_CH4_AF3 = 0x03,

  // TIM9 
  PA2_TIM9_CH1_AF3 = 0x03,
  PA3_TIM9_CH2_AF3 = 0x03,

  // TIM10
  PB8_TIM10_CH1_AF3 = 0x03,

  // TIM11
  PB9_TIM11_CH1_AF3 = 0x03,

  // TIM12
  PB14_TIM12_CH1_AF9 = 0x09,
  PB15_TIM12_CH2_AF9 = 0x09,

  // TIM13
  PA6_TIM13_CH1_AF9 = 0x09,

  // TIM14
  PA7_TIM14_CH1_AF9 = 0x09,
  #pragma endregion Timer Alternative Function Selection

  #pragma region I2C Alternative Function Selection
  // I2C-1
  PB5_I2C1_SMB_AF4 = 0x04,
  PB6_I2C1_SCL_AF4 = 0x04,
  PB7_I2C1_SDA_AF4 = 0x04,
  PB8_I2C1_SCL_AF4 = 0x04,
  PB9_I2C1_SDA_AF4 = 0x04,

  // I2C-2
  PB12_I2C2_SMBA_AF4 = 0x04,
  PB10_I2C2_SCL_AF4 = 0x04,
  PB11_I2C2_SDA_AF4 = 0x04,

  // I2C-3
  PA9_I2C3_SMBA_AF4 = 0x04,
  PA8_I2C3_SCL_AF4 = 0x04,
  PC9_I2C2_SDA_AF4 = 0x04,
  #pragma endregion I2C Alternative Function Selection

  #pragma region SPI/I2S Alternative Function Selection
  // SPI1
  PA4_SPI1_NSS_AF5 = 0x05,
  PA5_SPI1_SCK_AF5 = 0x05,
  PA6_SPI1_MISO_AF5 = 0x05,
  PA7_SPI1_MOSI_AF5 = 0x05,
  PA15_SPI1_NSS_AF5 = 0x05,
  PB3_SPI1_SCK_AF5 = 0x05,
  PB4_SPI1_MISO_AF5 = 0x05,
  PB5_SPI1_MOSI_AF5 = 0x05,

  // SPI2
  PB9_SPI2_NSS_AF5 = 0x05,
  PB10_SPI2_SCK_AF5 = 0x05,
  PB12_SPI2_NSS_AF5 = 0x05,
  PB13_SPI2_SCK_AF5 = 0x05,
  PB14_SPI2_MISO_AF5 = 0x05,
  PB15_SPI2_MOSI_AF5 = 0x05,
  PC2_SPI2_MISO_AF5 = 0x05,
  PC3_SPI2_MOSI_AF5 = 0x05,

  // SPI3
  PA4_SPI3_NSS_AF6 = 0x06,
  PA15_SPI3_NSS_AF6 = 0x06,
  PB3_SPI3_SCK_AF6 = 0x06,
  PB4_SPI3_MISO_AF6 = 0x06,
  PB5_SPI3_MOSI_AF6 = 0x06,
  PC10_SPI3_SCK_AF6 = 0x06,
  PC11_SPI3_MISO_AF6 = 0x06,
  PC12_SPI3_MOSI_AF6 = 0x06,

  // I2S2
  PC9_I2S_CKIN_AF5 = 0x05,
  PB9_I2S2_WS_AF5 = 0x05,
  PB10_I2S2_CK_AF5 = 0x05,
  PB12_I2S2_WS_AF5 = 0x05,
  PB13_I2S2_CK_AF5 = 0x05,
  PB14_I2S2ext_SD_AF6 = 0x06,
  PB15_I2S2_SD_AF5 = 0x05,
  PC2_I2S2ext_SD_AF6 = 0x06,
  PC3_I2S2_SD_AF5 = 0x05,
  PC6_I2S2_MCK_AF5 = 0x05,

  // I2S3
  PA4_I2S3_WS_AF6 = 0x06,
  PA15_I2S3_WS_AF6 = 0x06,
  PB3_I2S3_CK_AF6 = 0x06,
  PB4_I2S3ext_SD_AF7 = 0x07,
  PB5_I2S3_SD_AF6 = 0x06,
  PC7_I2S3_MCK_AF6 = 0x06,
  PC10_I2S3_CK_AF6 = 0x06,
  PC11_I2S3ext_SD_AF5 = 0x05,
  PC12_I2S3_SD_AF6 = 0x06,
  #pragma endregion SPI/I2S Alternative Function Selection
  
  #pragma region USART Alternative Function Selection
  // USART1
  PA8_USART1_CK_AF7 = 0x07,
  PA9_USART1_TX_AF7 = 0x07,
  PA10_USART1_RX_AF7 = 0x07,
  PA11_USART1_CTS_AF7 = 0x07,
  PA12_USART1_RTS_AF7 = 0x07,
  PB6_USART1_TX_AF7 = 0x07,
  PB7_USART1_RX_AF7 = 0x07,

  // USART2
  PA0_USART2_CTS_AF7 = 0x07,
  PA1_USART2_RTS_AF7 = 0x07,
  PA2_USART2_TX_AF7 = 0x07,
  PA3_USART2_RX_AF7 = 0x07,
  PA4_USART2_CK_AF7 = 0x07,

  // USART3
  PB13_USART3_CTS_AF7 = 0x07,
  PB14_USART3_RTS_AF7 = 0x07,
  PB10_USART3_TX_AF7 = 0x07,
  PB11_USART3_RX_AF7 = 0x07,
  PB12_USART3_CK_AF7 = 0x07,
  PC10_USART3_TX_AF7 = 0x07,
  PC11_USART3_RX_AF7 = 0x07,
  PC12_USART3_CK_AF7 = 0x07,

  // UART4
  PA0_UART4_TX_AF8 = 0x08,
  PA1_UART4_RX_AF8 = 0x08,
  PC10_UART4_TX_AF8 = 0x08,
  PC11_UART4_RX_AF8 = 0x08,

  // UART5
  PC12_UART5_TX_AF8 = 0x08,
  PD2_UART5_RX_AF8 = 0x08,

  // USART6
  PC6_USART6_TX_AF8 = 0x08,
  PC7_USART6_RX_AF8 = 0x08,
  PC8_USART6_CK_AF8 = 0x08,
  #pragma endregion USART Alternative Function Selection

  #pragma region CAN Alternative Function Selection
  // CAN1
  PA11_CAN1_RX_AF9 = 0x09,
  PA12_CAN1_TX_AF9 = 0x09,
  PB8_CAN1_RX_AF9 = 0x09,
  PB9_CAN1_TX_AF9 = 0x09,

  // CAN2
  PB5_CAN2_RX_AF9 = 0x09,
  PB6_CAN2_TX_AF9 = 0x09,
  PB12_CAN2_RX_AF9 = 0x09,
  PB13_CAN2_TX_AF9 = 0x09,
  #pragma endregion CAN Alternative Function Selection

  #pragma region USB
  // HS
  PA3_OTG_HS_ULPI_D0_AF10 = 0x0A,
  PA5_OTG_HS_ULPI_CK_AF10 = 0x0A,
  PB0_OTG_HS_ULPI_D1_AF10 = 0x0A,
  PB1_OTG_HS_ULPI_D2_AF10 = 0x0A,
  PB10_OTG_HS_ULPI_D3_AF10 = 0x0A,
  PB11_OTG_HS_ULPI_D4_AF10 = 0x0A,
  PB12_OTG_HS_ULPI_D5_AF10 = 0x0A,
  PB13_OTG_HS_ULPI_D6_AF10 = 0x0A,
  PB5_OTG_HS_ULPI_D7_AF10 = 0x0A,
  PC0_OTG_HS_ULPI_STP_AF10 = 0x0A,
  PC2_OTG_HS_ULPI_DIR_AF10 = 0x0A,
  PC3_OTG_HS_ULPI_NXT_AF10 = 0x0A,
  PB12_OTG_HS_ID_AF12 = 0x0C,
  PB14_OTG_HS_DM_AF12 = 0x0C,
  PB15_OTG_HS_DP_AF12 = 0x0C,

  // FS
  PA8_OTG_FS_SOF_AF10 = 0x0A,
  PA10_OTG_FS_ID_AF10 = 0x0A,
  PA11_OTG_FS_DM_AF10 = 0x0A,
  PA12_OTG_FS_DP_AF10 = 0x0A,  
  #pragma endregion USB

  #pragma region ETH
  PA0_ETH_MII_CRS_AF11 = 0x0B,
  PA1_ETH_MII_RX_CLK_AF11 = 0x0B,
  PA1_ETH_RMII_REF_CLK_AF11 = 0x0B,
  PA2_ETH_MDIO_AF11 = 0x0B,
  PA3_ETH_MII_COL_AF11 = 0x0B,
  PA7_ETH_MII_RX_DV_AF11 = 0x0B,
  PA7_ETH_RMII_SRS_DV_AF11 = 0x0B,
  PB0_ETH_MII_RXD2_AF11 = 0x0B,
  PB1_ETH_MII_RXD3_AF11 = 0x0B,
  PB5_ETH_PPS_OUT_AF11 = 0x0B,
  PB8_ETH_MII_TXD3_AF11 = 0x0B,
  PB10_ETH_MII_RX_ER_AF11 = 0x0B,
  PB11_ETH_MII_TX_EN_AF11 = 0x0B,
  PB11_ETH_RMII_TX_EN_AF11 = 0x0B,
  PB12_ETH_MII_TXD0_AF11 = 0x0B,
  PB12_ETH_RMII_TXD0_AF11 = 0x0B,
  PB13_ETH_MII_TXD1_AF11 = 0x0B,
  PB13_ETH_RMII_TXD1_AF11 = 0x0B,
  PC1_ETH_MDC_AF11 = 0x0B,
  PC2_ETH_MII_TXD2_AF11 = 0x0B,
  PC3_ETH_MII_TX_CLK_AF11 = 0x0B,
  PC4_ETH_MII_RXD0_AF11 = 0x0B,
  PC4_ETH_RMII_RXD0_AF11 = 0x0B,
  PC5_ETH_MII_RXD1_AF11 = 0x0B,
  PC5_ETH_RMII_RXD1_AF11 = 0x0B,
  #pragma endregion ETH

  #pragma region _SDIO
  PB8_SDIO_D4_AF12 = 0x0C,
  PB9_SDIO_D5_AF12 = 0x0C,
  PC6_SDIO_D6_AF12 = 0x0C,
  PC7_SDIO_D7_AF12 = 0x0C,
  PC8_SDIO_D0_AF12 = 0x0C,
  PC9_SDIO_D1_AF12 = 0x0C,
  PC10_SDIO_D2_AF12 = 0x0C,
  PC11_SDIO_D3_AF12 = 0x0C,
  PC12_SDIO_CK_AF12 = 0x0C,
  PD2_SDIO_CMD_AF12 = 0x0C,
  #pragma endregion _SDIO

  #pragma region DCMI
  PA4_DCMI_HSYNC_AF13 = 0x0D,
  PA6_DCMI_PIXCK_AF13 = 0x0D,
  PA9_DCMI_D0_AF13 = 0x0D,
  PA10_DCMI_D1_AF13 = 0x0D,
  PB5_DCMI_D10_AF13 = 0x0D,
  PB6_DCMI_D5_AF13 = 0x0D,
  PB7_DCMI_VSYNC_AF13 = 0x0D,
  PB8_DCMI_D6_AF13 = 0x0D,
  PB9_DCMI_D7_AF13 = 0x0D,
  PC6_DCMI_D0_AF13 = 0x0D,
  PC7_DCMI_D1_AF13 = 0x0D,
  PC8_DCMI_D2_AF13 = 0x0D,
  PC9_DCMI_D3_AF13 = 0x0D,
  PC10_DCMI_D8_AF13 = 0x0D,
  PC11_DCMI_D4_AF13 = 0x0D,
  PC12_DCMI_D9_AF13 = 0x0D,
  PD2_DCMI_D11_AF13 = 0x0D,
  #pragma endregion DCMI
  ALL_AF15_EVENTOUT = 0x0F
}AFMode;

#ifdef __cplusplus
}
#endif

#endif