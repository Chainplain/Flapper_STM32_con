/**
	this is for UART RS232

	created by 2020-07-10

	using DMA for tx and rx

	using interrupt for tx

	using polling check method for rx

**/

#include "main.h"
#include "uart_232.h"
#include "interface_uart.h"

#define USE_UART_NUM 1
UART232_Handle *uart_handle[USE_UART_NUM];
UART_HandleTypeDef UartHandle;

void vTaskUart(void *pvParameters)
{
	uart_init();

	while (true)
	{
		for (int i = 0; i < USE_UART_NUM; i++)
		{
			if (uart_handle[i] != NULL)
			{	
				// osMutexWait(sendUartMutexId, osWaitForever);
				char data[] = "Hello, UART!\n";
				uint8_t uintArray[sizeof(data)];
				for (int i = 0; i < sizeof(data); i++) {
       			 uintArray[i] = (uint8_t)data[i];
  			    }
				// uintArray[0]  = sizeof(data);
				uart_handle[i]->huart->pTxBuffPtr = uintArray;
				uart_tx_dma_start(uart_handle[i]->huart,\
								 sizeof(uintArray));
				delay(100);
				// uart_handle[i]->func_rx_check(uart_handle[i]);
			}
				
		}
		vTaskDelayMs(RX_ROLLING_INTERVEL_MS);
	}

	// 如果任务不是永久性的需要调用 vTaskDelete 删除任务
	//vTaskDelete(NULL);
}

void uart_init(void)
{
	uart_handle[0] = rs232_computer_init();
}

void UART_PP_Init(UART_HandleTypeDef *huart, uint32_t baudrate, uint32_t stopbit, uint32_t parity, uint32_t bufsize)
{
	// ##-1- Configure the UART peripheral ######################################
	// Put the USART peripheral in the Asynchronous mode (UART Mode)
	// UART1 configured as follow:
	//    - Word Length = 8 Bits
	//    - Stop Bit    2->2 bit, others->1 bit
	//    - Parity      0->none, 1->odd, 2->even
	//    - BaudRate
	//    - Hardware flow control disabled (RTS and CTS signals)
	huart->Init.BaudRate = baudrate;
	huart->Init.WordLength = UART_WORDLENGTH_8B;
	if (stopbit == 2)
		huart->Init.StopBits = UART_STOPBITS_2;
	else
		huart->Init.StopBits = UART_STOPBITS_1;
	if (parity == 0)
		huart->Init.Parity = UART_PARITY_NONE;
	else if (parity == 1)
		huart->Init.Parity = UART_PARITY_ODD;
	else
		huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart->Init.Mode = UART_MODE_TX_RX;
	huart->Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(huart);

	uart_dma_init(huart);

	HAL_UART_Receive_DMA(huart, huart->pRxBuffPtr, bufsize);
}

void uart_dma_init(UART_HandleTypeDef *huart)
{
	DMA_HandleTypeDef *hdma_tx;
	DMA_HandleTypeDef *hdma_rx;

	hdma_tx = huart->hdmatx;
	hdma_rx = huart->hdmarx;

	// ##-3- Configure the DMA streams ##########################################
	// Configure the DMA handler for Transmission process
	// hdma_tx->Instance                 = DMA1_Channel4; // refer to reference manual page 299
	// hdma_tx->Init.Request             = DMA_REQUEST_2;
	hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_tx->Init.MemInc = DMA_MINC_ENABLE;
	hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_tx->Init.Mode = DMA_NORMAL;
	hdma_tx->Init.Priority = DMA_PRIORITY_LOW;
	HAL_DMA_DeInit(hdma_tx);
	HAL_DMA_Init(hdma_tx);

	// Configure the DMA handler for reception process
	// hdma_rx->Instance                 = DMA1_Channel5; // refer to reference manual page 299
	// hdma_rx->Init.Request             = DMA_REQUEST_2;
	hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_rx->Init.MemInc = DMA_MINC_ENABLE;
	hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_rx->Init.Mode = DMA_CIRCULAR;
	hdma_rx->Init.Priority = DMA_PRIORITY_HIGH;
	HAL_DMA_DeInit(hdma_rx);
	HAL_DMA_Init(hdma_rx);
}

/**  上面是初始化， 下面是工作方式
** -1- 采用抢占是发送方式，即自动覆盖上一次发送数据
** -2- 采用轮询式串口检查方式
**/

void uart_tx_dma_start(UART_HandleTypeDef *uartP, uint32_t len)
{
	// 清空标志位，不然无法正常使用 HAL_UART_Transmit_DMA 函数
	uartP->gState = HAL_UART_STATE_READY;
	uartP->hdmatx->State = HAL_DMA_STATE_READY;
	uartP->Lock = HAL_UNLOCKED;
	uartP->hdmatx->Lock = HAL_UNLOCKED;

	HAL_UART_Transmit(uartP, uartP->pTxBuffPtr, len, 100);
}

/**  采用 桢头 + 包长度 的通讯协议
 **/
void uart_rx_data_check_withFrameHeaderAndLength(UART232_Handle *uart232P)
{
	uint32_t dmaRemCnt;
	UART_FRAME *frame;
	uint8_t dataTmp;
	uint32_t tmp;
	uint8_t *rxBuffer = uart232P->rxBuffer;

	uint32_t i;

	dmaRemCnt = uart232P->huart->hdmarx->Instance->NDTR;
	if (dmaRemCnt == 0)
		dmaRemCnt = uart232P->rxBufferSize; // 代表有数据没有读取

	// 检查有无数据读入
	if (uart232P->unReadIndex != dmaRemCnt)
	{
		//     计算长度
		uart232P->toBeReadLength = uart232P->unReadIndex + uart232P->rxBufferSize - dmaRemCnt;
		if (uart232P->toBeReadLength > uart232P->rxBufferSize)
			uart232P->toBeReadLength -= uart232P->rxBufferSize;

		frame = &uart232P->rxFrame;

		while (1)
		{
			// 读数
			dataTmp = uart232P->huart->pRxBuffPtr[uart232P->rxBufferSize - uart232P->unReadIndex];
			if (frame->index >= PROTOCOL_BUFFER_SIZE)
				frame->index = 0;

			if (uart232P->unReadIndex <= 1)
				uart232P->unReadIndex = uart232P->rxBufferSize;
			else
				uart232P->unReadIndex--;

			if (frame->index < frame->headerLen)
			{ // 处于桢头位置，需要判断桢头是否正确
				if (dataTmp != frame->header[frame->index])
				{ // 如果桢头对应不上，从当前字节开始认为下一个桢的起点
					if (dataTmp != frame->header[0])
					{
						frame->index = 0;
					}
					else
					{
						rxBuffer[0] = dataTmp;
						frame->index++;
					}
				}
				else
				{
					rxBuffer[frame->index] = dataTmp;
					frame->index++;
				}
			}
			else
			{ // 如果不是桢头，不需要甄别，直接存入数组中，然后在分析
				rxBuffer[frame->index] = dataTmp;
				frame->index++;
				if (frame->index > uart232P->rxBufferSize)
				{ // 数据溢出
					frame->index = 0;
				}

				// tmp = 包前头的所有长度 (包括 帧头+命令+数据长度)
				tmp = frame->headerLen + frame->cmdLen + frame->dataNumLen;
				if (frame->index < tmp)
				{ // 桢还没有发完，什么也不要做
				}
				else if (frame->index == tmp)
				{ // 计算 桢的长度
					tmp = 0;
					if (frame->dataNumMsbFirst == 0) // 代表长度的数据数组中，高字节在后
					{
						for (i = 0; i < frame->dataNumLen; i++)
						{
							tmp += ((rxBuffer[frame->index - frame->dataNumLen + i]) << (i * 8));
						}
					}
					else // 代表长度的数据数组中，高字节在前
					{
						for (i = 0; i < frame->dataNumLen; i++)
						{
							tmp += ((rxBuffer[frame->index - frame->dataNumLen + i]) << ((frame->dataNumLen - i - 1) * 8));
						}
					}
					// 帧头+命令+数据长度  +  数据  +  校验
					frame->dataLen = tmp;
					frame->length = frame->index + frame->dataLen + frame->checkLen;
				}
				else
				{
					if (frame->length == frame->index)
					{ // 一个桢结束了
						// go to process function
						if (uart232P->func_data_analysis != NULL)
						{
							if (uart232P->func_crc_check(uart232P) == 0) // 校验OK
							{
								uart232P->func_data_analysis(uart232P);
							}
						}

						frame->index = 0; // 开始接受新的桢
					}
				}
			}

			// 所有未读的数都处理完毕
			if (uart232P->unReadIndex == dmaRemCnt)
				break;
		}
	}
}
