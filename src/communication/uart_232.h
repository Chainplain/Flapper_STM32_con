

#ifndef _UART_232_H
#define _UART_232_H

#include <stm32f4xx_hal.h>

#define RX_ROLLING_INTERVEL_MS	1  // 轮询的时间间隔

typedef struct __UART_FRAME
{
	uint8_t *header; // Frame头的匹配字符地址
	uint32_t headerLen; // Frame头长度
	uint32_t cmdLen; // FrameCMD占用的字节数
	uint8_t *cmdPtr;
	uint32_t dataNumLen; // Frame数据长度占用的字节数
	uint8_t dataNumMsbFirst; // 是否是先高字节后低字节
	uint32_t dataLen; // Frame数据占用的字节数	
	uint8_t *dataPtr;
	uint32_t checkLen; // 检验位占用的字节数
	uint8_t checkMsbFirst; // 是否是先高字节后低字节
	uint32_t index; // 代表Frame长度的字节索引
	uint32_t length; // 代表Frame长度的字节数
}UART_FRAME;

typedef struct __UART232_Handle
{
	UART_HandleTypeDef *huart;
	
	// 与DMA有关的变量
	uint32_t unReadIndex; // 代表一个有效Frame的第一个数据的位置
	uint32_t unCheckIndex; // 代表一个有效Frame的最后一个数据的位置
	uint16_t toBeReadLength;
	
	uint32_t txBufferSize;
	uint32_t rxBufferSize;	
	uint8_t *txCmdBuffer;
	uint8_t *txDataBuffer;
	uint8_t *txBuffer;
	uint8_t *rxBuffer;
	
	void (* func_data_analysis) (struct __UART232_Handle * hrs232);
	void (* func_rx_check) (struct __UART232_Handle * hrs232);
	int (* func_crc_check) (struct __UART232_Handle * hrs232);
	UART_FRAME rxFrame;
	UART_FRAME txFrame;
}UART232_Handle;

// float    32bit,  31          30 23         22 0
//                  sign          E            M
//        val = (-1)^sign *  2^(E - 127)  *  (1 + M)
typedef struct
{
	uint8_t u1;
	uint8_t u2;
	uint8_t u3;
	uint8_t u4;
}FLOATBYTE_STRUCT;

typedef union
{
	float fval;
	FLOATBYTE_STRUCT uval;
	uint32_t val32;
}FLOAT32_UNION;

typedef struct
{
	uint8_t u1;
	uint8_t u2;
	uint8_t u3;
	uint8_t u4;
	uint8_t u5;
	uint8_t u6;
	uint8_t u7;
	uint8_t u8;
}DOUBLEBYTE_STRUCT;

typedef union
{
	double dval;
	DOUBLEBYTE_STRUCT luval;
	uint64_t val64;
}DOUBLE64_UNION;

void uart_init(void);

void UART_PP_Init(UART_HandleTypeDef *huart, uint32_t baudrate, uint32_t stopbit, uint32_t parity, uint32_t bufsize);
void uart_dma_init(UART_HandleTypeDef *huart);

void uart_tx_dma_start(UART_HandleTypeDef* uartP, uint32_t len);

void uart_rx_data_check_withFrameHeaderAndLength(UART232_Handle *uart232P);

void vTaskUart(void *pvParameters);

#endif
