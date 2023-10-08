#include "uart_esp.h"
#include "main.h"
#include "fifo.h"
#include "userString.h"

#define SSID "Xiaomi_Haivoo"
#define PASSWORD "jinsheng"
#define IP "192.168.31.202"
#define PORT (6001U)

#define RXDATA_LENGTH (1024)
static uint8_t RxData[RXDATA_LENGTH];
static volatile int RxDataIndex = 0;
static int uart_rx_data(uint8_t *data, int length);
// -1: wifi 失败
// 0: tcp 失败
// 1: tcp 成功
volatile int EspFlag = -1;

osThreadId esp_uart_thread_id;

static uint8_t sendFifoMemory[PLOT_FIFO_SIZE];
static FIFOClass sendFifo(PLOT_FIFO_SIZE, sendFifoMemory); // 10k bytes
static osMutexId sendUartMutexId;
static osThreadId sendEspUart_thread_id;

static __IO bool sendValid = false;
#define USE_UART3 1
#define USART_COMPUTER USART3
#define USART_COMPUTER_CLOCK_ENABLE() __HAL_RCC_USART3_CLK_ENABLE()
#define USART_COMPUTER_DMA_CLOCK_ENABLE() __HAL_RCC_DMA1_CLK_ENABLE()
#define USART_COMPUTER_TX_DMA_CHANNEL DMA_CHANNEL_4 // refer to reference manual page 307 - 308
#define USART_COMPUTER_RX_DMA_CHANNEL DMA_CHANNEL_4
#define USART_COMPUTER_TX_DMA_STREAM DMA1_Stream3
#define USART_COMPUTER_RX_DMA_STREAM DMA1_Stream1

static UART_HandleTypeDef UartHandle;
static DMA_HandleTypeDef UartDmaTxHandle, UartDmaRxHandle;
#define RXBUFFERSIZE PROTOCOL_BUFFER_SIZE
#define TXBUFFERSIZE PROTOCOL_BUFFER_SIZE
static uint8_t TxMemory[RXBUFFERSIZE]; // for DMA Tx : user buffer -> frame buffer -> fifo buffer -> DMA buffer(TxMemory)
static uint8_t RxMemory[RXBUFFERSIZE]; // for DMA Rx
static uint8_t RxBuffer[RXBUFFERSIZE]; // rx data flow : DMA buffer (RxMemory) -> Rx buffer -> frame buffer
static UART232_Handle uart232_handle, *uart_esp_handle;

static void uart_esp_init(uint32_t baudrate);
static void sendTaskInit(void);
static void uart_rx_check(UART232_Handle *uart232P);

UART232_Handle *rs232_esp_init(void)
{
	uart_esp_handle = &uart232_handle;
	uart_esp_init(115200);
	return uart_esp_handle;
}

static void uart_esp_init(uint32_t baudrate)
{
	// ----------------------------------------------
	// USART instance initializing
	// ----------------------------------------------
	// 此处需要在 头文件里 更改
	USART_COMPUTER_CLOCK_ENABLE();
	USART_COMPUTER_DMA_CLOCK_ENABLE();							  // 打开串口GPIO时钟
	UartHandle.Instance = USART_COMPUTER;						  // USART1
	UartHandle.Instance = USART_COMPUTER;						  // USART3
	UartDmaTxHandle.Instance = USART_COMPUTER_TX_DMA_STREAM;	  // DMA1_Stream3
	UartDmaTxHandle.Init.Channel = USART_COMPUTER_TX_DMA_CHANNEL; // DMA_CHANNEL_4
	UartDmaRxHandle.Instance = USART_COMPUTER_RX_DMA_STREAM;	  // DMA1_Stream1
	UartDmaRxHandle.Init.Channel = USART_COMPUTER_RX_DMA_CHANNEL; // DMA_CHANNEL_4

	UartHandle.pTxBuffPtr = TxMemory;
	UartHandle.pRxBuffPtr = RxMemory;
	__HAL_LINKDMA(&UartHandle, hdmatx, UartDmaTxHandle);
	__HAL_LINKDMA(&UartHandle, hdmarx, UartDmaRxHandle);

	// 此处需要 根据 实际情况更改
	UART_PP_Init(&UartHandle, baudrate, 1, 0, RXBUFFERSIZE);

	uart232_handle.huart = &UartHandle;
	uart232_handle.func_rx_check = uart_rx_check;
	uart232_handle.unCheckIndex = uart232_handle.huart->hdmarx->Instance->NDTR;
	uart232_handle.unReadIndex = uart232_handle.huart->hdmarx->Instance->NDTR;
	uart232_handle.toBeReadLength = 0;

	uart232_handle.rxBufferSize = RXBUFFERSIZE;
	uart232_handle.txBufferSize = TXBUFFERSIZE;
	uart232_handle.rxBuffer = RxBuffer;

	osDelay(100);
	HAL_GPIO_WritePin(REST_PORT, REST_PIN, GPIO_PIN_SET);
	osDelay(1500);

	sendTaskInit();
}

static void vTaskUartSend(void *pvParameters)
{
	int len;
	sendValid = true;
	while (1)
	{
		if (uart_esp_handle->huart->hdmatx->Instance->NDTR == 0) // 代表DMA处于空闲状态
		{
			osMutexWait(sendUartMutexId, osWaitForever);
			len = sendFifo.occupiedSize();
			// 代表有数据要发送
			if (len)
			{
				len = (len > TXBUFFERSIZE) ? TXBUFFERSIZE : len;
				len = sendFifo.read(uart_esp_handle->huart->pTxBuffPtr, len);
				uart_tx_dma_start(uart_esp_handle->huart, len);
			}
			osMutexRelease(sendUartMutexId);
		}

		vTaskDelayMs(1);
	}

	// 如果任务不是永久性的需要调用 vTaskDelete 删除任务
	// vTaskDelete(NULL);
}

static void sendTaskInit(void)
{
	// 创建 send 任务
	// 创建 互斥锁
	osMutexDef(uartSend);
	sendUartMutexId = osMutexCreate(osMutex(uartSend)); // 要在任务之前设置，不然可能未初始化就被使用

	osThreadDef(uartSend, vTaskUartSend, osPriority::osPriorityNormal, 0, 256);
	sendEspUart_thread_id = osThreadCreate(osThread(uartSend), NULL);
}

uint8_t uart_esp_send(uint8_t *data, int length)
{
	while (!sendValid)
	{ // 代表send口还没有准备好
		delay(1);
	}
	osMutexWait(sendUartMutexId, osWaitForever);
	while (sendFifo.remainedSize() < (int)length)
	{ // 等待其他任务释放fifo空间
		osMutexRelease(sendUartMutexId);
		osDelay(1);
		osMutexWait(sendUartMutexId, osWaitForever);
	}

	sendFifo.write(data, length);

	osMutexRelease(sendUartMutexId);

	return 0;
}

uint8_t uart_esp_send_str(char *sbuf)
{
	int len = strlen(sbuf);
	if (len > 250)
		return 1;
	return uart_esp_send((uint8_t *)sbuf, len);
}

static void uart_print2com(uint8_t *data, int length)
{
	if ((length > 0) && (length < 240))
		userComm->send('s', data, length);
}

static void uart_rx_check(UART232_Handle *uart232P)
{
	const int dmaRemCnt = uart232P->huart->hdmarx->Instance->NDTR;
	const int preIndex = uart232P->unReadIndex; // 上一次的dma指针位置

	int len = preIndex - dmaRemCnt;
	if (len != 0) // 代表有数据
	{
		// dmaRemCnt是从大到小变的，到0后重置为最大
		int index = RXBUFFERSIZE - preIndex;
		if (index < 0)
			index += RXBUFFERSIZE;

		if (len >= 0) // 如果没有头尾相接
		{
			for (int i = 0; i < len; i++)
			{
				RxBuffer[i] = RxMemory[index++];
			}
		}
		else // 头尾相接，走了一圈
		{
			for (int i = 0; i < preIndex; i++)
			{
				RxBuffer[i] = RxMemory[index++];
			}
			len = RXBUFFERSIZE - dmaRemCnt;
			int bufIndx = preIndex;
			for (int i = 0; i < len; i++)
			{
				RxBuffer[bufIndx++] = RxMemory[i];
			}
		}

		uart232P->unReadIndex = dmaRemCnt;

		len = preIndex - dmaRemCnt;
		if (len < 0)
			len += RXBUFFERSIZE;

		uart_rx_data(RxBuffer, len);
		uart_print2com(RxBuffer, len); // 回读, 并打印到电脑串口
	}
}

static int uart_rx_data(uint8_t *data, int length)
{
	for (int i = 0; i < length; i++)
	{
		if (RxDataIndex >= RXDATA_LENGTH)
			RxDataIndex = 0;

		RxData[RxDataIndex++] = data[i];
	}
	return RxDataIndex;
}
