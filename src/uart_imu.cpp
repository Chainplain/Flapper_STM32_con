/**
	this is for UART 232 code interpret

	created by 2019-12-28

**/

/**

**/

#include "uart_imu.h"
#include "main.h"

IMU_TypeOf_Handle imuHandle;

#define USE_UART4 1
#define USART_IMU UART4
#define USART_IMU_CLOCK_ENABLE() __HAL_RCC_UART4_CLK_ENABLE()
#define USART_IMU_DMA_CLOCK_ENABLE() __HAL_RCC_DMA1_CLK_ENABLE()
#define USART_IMU_TX_DMA_CHANNEL DMA_CHANNEL_4 // refer to reference manual page 299
#define USART_IMU_RX_DMA_CHANNEL DMA_CHANNEL_4
#define USART_IMU_TX_DMA_STREAM DMA1_Stream4
#define USART_IMU_RX_DMA_STREAM DMA1_Stream2

// -------------------- 基本 --------------------------------
#define UART_FRAME_HEADER_LENGTH 2
#define TXBUFFERSIZE 256
#define RXBUFFERSIZE 256
#define CMDSIZE 1
static uint8_t UartFrameHeader[UART_FRAME_HEADER_LENGTH] = {0x55, 0x55};

static UART_HandleTypeDef UartHandle;
static DMA_HandleTypeDef UartDmaTxHandle, UartDmaRxHandle;
static uint8_t TxBuffer[TXBUFFERSIZE];
static uint8_t RxBuffer[RXBUFFERSIZE];
static uint8_t TxDataBuffer[TXBUFFERSIZE];
static uint8_t TxCmdBuffer[CMDSIZE];
static uint8_t TxMemory[TXBUFFERSIZE];
static uint8_t RxMemory[RXBUFFERSIZE];

static uint8_t *RS232_DATA;
static UART232_Handle uart232_handle, *rs232_handle;

// -----------------------------------------------------
// ------------------ 语法 -----------------------------------------
static int uart_rx_crc_check(UART232_Handle *rs232);
static UART232_Handle *rs232_com_init(uint32_t baudrate);
static void rs232_data_analysis(UART232_Handle *rs232);
// static void rs232_uart_send(void);
// static uint8_t *rs232_tx_data_buf_get(void);
// static void uartPrintf(char *data);
// static void uartStrCat(char *des, const char *src, int8_t rneeded);

enum RS232_COMMAND
{
	IMUCMD_SAVE = 0,
	ZITAI_CMD = 0x01,
	SIYUANSHU_CMD = 0x02,
	GYRO_ACC_CMD = 0x03,
	MAGNET_CMD = 0x04,
	PRESSURE_CMD = 0x05,
	PORT_STATUS_CMD = 0x06,
	IMUCMD_BAUD = 0x07,
	IMUCMD_RETURNSET = 0x88,
	IMUCMD_RETURNRATE = 0x0A,
	IMUCMD_ALG = 0x0B,
	IMUCMD_RESET = 0x7F,
	IMUCMD_GYRO_RANGE = 0x83,
	IMUCMD_ACC_RANGE = 0x84,
};
static void rs232_data_zitai(void);
static void rs232_data_siyuanshu(void);
static void rs232_data_gyro_acc(void);
static void rs232_data_magnet(void);
static void rs232_data_pressure(void);

// 发送部分函数声明
static uint32_t imu_uart_frame_construct(void);

UART232_Handle *rs232_imu_init(void)
{
	rs232_handle = &uart232_handle;
	rs232_com_init(115200);
	return rs232_handle;
}

// 该函数替代了uart_rx_data_check_withEndCharacter作为循环轮询的入口函数
//    增加了SX1278的各个状态 和 省电模式的轮询
static void uart_process(UART232_Handle *uart232P)
{
	uart_rx_data_check_withFrameHeaderAndLength(uart232P);
}

static UART232_Handle *rs232_com_init(uint32_t baudrate)
{
	// ----------------------------------------------
	// USART instance initializing
	// ----------------------------------------------
	// 此处需要在 头文件里 更改
	USART_IMU_CLOCK_ENABLE();
	USART_IMU_DMA_CLOCK_ENABLE();							 // 打开串口GPIO时钟
	UartHandle.Instance = USART_IMU;						 // USART1
	UartDmaTxHandle.Instance = USART_IMU_TX_DMA_STREAM;		 // DMA1_Stream3
	UartDmaTxHandle.Init.Channel = USART_IMU_TX_DMA_CHANNEL; // DMA_CHANNEL_4
	UartDmaRxHandle.Instance = USART_IMU_RX_DMA_STREAM;		 // DMA1_Stream1
	UartDmaRxHandle.Init.Channel = USART_IMU_RX_DMA_CHANNEL; // DMA_CHANNEL_4

	UartHandle.pTxBuffPtr = TxMemory;
	UartHandle.pRxBuffPtr = RxMemory;
	__HAL_LINKDMA(&UartHandle, hdmatx, UartDmaTxHandle);
	__HAL_LINKDMA(&UartHandle, hdmarx, UartDmaRxHandle);

	// 此处需要 根据 实际情况更改
	UART_PP_Init(&UartHandle, baudrate, 1, 0, RXBUFFERSIZE);
	//__HAL_DMA_DISABLE(uart232_handle.huart->hdmarx);
	// uart232_handle.huart->hdmarx->Instance->NDTR = RXBUFFERSIZE;
	//__HAL_DMA_ENABLE(uart232_handle.huart->hdmarx);

	// 此处需要根据 实际协议 更改
	uart232_handle.huart = &UartHandle;
	uart232_handle.txCmdBuffer = TxCmdBuffer;
	uart232_handle.txDataBuffer = TxDataBuffer;
	uart232_handle.rxBufferSize = RXBUFFERSIZE;
	uart232_handle.txBufferSize = TXBUFFERSIZE;

	uart232_handle.func_data_analysis = rs232_data_analysis;
	uart232_handle.func_rx_check = uart_process;
	uart232_handle.func_crc_check = uart_rx_crc_check;

	uart232_handle.rxBuffer = RxBuffer;
	uart232_handle.txBuffer = TxBuffer;
	uart232_handle.rxFrame.header = UartFrameHeader;
	uart232_handle.rxFrame.headerLen = UART_FRAME_HEADER_LENGTH;
	uart232_handle.rxFrame.cmdLen = CMDSIZE;
	uart232_handle.rxFrame.dataNumLen = 1;
	uart232_handle.rxFrame.dataNumMsbFirst = 1; // 先高字节，后低字节
	uart232_handle.rxFrame.dataLen = 0;
	uart232_handle.rxFrame.checkLen = 1;
	uart232_handle.rxFrame.checkMsbFirst = 1; // 先高字节，后低字节
	uart232_handle.rxFrame.index = 0;
	uart232_handle.rxFrame.length = 0;
	uart232_handle.rxFrame.cmdPtr = &uart232_handle.rxBuffer[uart232_handle.rxFrame.headerLen];
	uart232_handle.rxFrame.dataPtr = uart232_handle.rxFrame.cmdPtr + uart232_handle.rxFrame.cmdLen + uart232_handle.rxFrame.dataNumLen;

	uart232_handle.txFrame.header = UartFrameHeader;
	uart232_handle.txFrame.headerLen = UART_FRAME_HEADER_LENGTH;
	uart232_handle.txFrame.cmdLen = CMDSIZE;
	uart232_handle.txFrame.dataNumLen = 1;
	uart232_handle.txFrame.dataNumMsbFirst = 1; // 先高字节，后低字节
	uart232_handle.txFrame.dataLen = 0;
	uart232_handle.txFrame.checkLen = 1;
	uart232_handle.txFrame.checkMsbFirst = 1; // 先高字节，后低字节
	uart232_handle.txFrame.index = 0;
	uart232_handle.txFrame.length = 0;
	uart232_handle.txFrame.cmdPtr = &uart232_handle.txBuffer[uart232_handle.txFrame.headerLen];
	uart232_handle.txFrame.dataPtr = uart232_handle.txFrame.cmdPtr + uart232_handle.txFrame.cmdLen + uart232_handle.txFrame.dataNumLen;

	// vTaskDelayMs(200); // RX线路稳定时间
	uart232_handle.unCheckIndex = uart232_handle.huart->hdmarx->Instance->NDTR;
	uart232_handle.unReadIndex = uart232_handle.huart->hdmarx->Instance->NDTR;
	uart232_handle.toBeReadLength = 0;

	RS232_DATA = uart232_handle.huart->pRxBuffPtr;

	return rs232_handle;
}

static int uart_rx_crc_check(UART232_Handle *rs232)
{
	int i;
	uint32_t check = 0;
	UART_FRAME *frame = &rs232->rxFrame;
	uint8_t *checkPtr = &rs232->rxBuffer[frame->length - frame->checkLen];
	if (frame->length > frame->checkLen)
	{
		int crclen = frame->length - frame->checkLen;

		for (i = 0; i < crclen; i++)
		{
			check += rs232->rxBuffer[i];
		}

		for (i = 0; i < (int)frame->checkLen; i++)
		{
			if (frame->checkMsbFirst)
			{
				if (checkPtr[i] != ((check >> ((frame->checkLen - i - 1) * 8)) & 0xFF))
				{
					return -1;
				}
			}
			else
			{
				if (checkPtr[i] != ((check >> ((i)*8)) & 0xFF))
				{
					return -1;
				}
			}
		}

		return 0;
	}
	else
		return -1;
}

static void rs232_data_analysis(UART232_Handle *rs232)
{
	switch (rs232->rxFrame.cmdPtr[0])
	{
	case ZITAI_CMD:
		rs232_data_zitai();
		break;
	case SIYUANSHU_CMD:
		rs232_data_siyuanshu();
		break;
	case GYRO_ACC_CMD:
		rs232_data_gyro_acc();
		break;
	case MAGNET_CMD:
		rs232_data_magnet();
		break;
	case PRESSURE_CMD:
		rs232_data_pressure();
		break;
	case PORT_STATUS_CMD:
		break;
	default:
		break;
	}
}

static void rs232_data_zitai(void)
{
	float zitai[3];
	uint8_t tmpH, tmpL;

	if (rs232_handle->rxFrame.dataLen == 6)
	{
		tmpL = rs232_handle->rxFrame.dataPtr[0];
		tmpH = rs232_handle->rxFrame.dataPtr[1];
		zitai[0] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f * 180.0f; // X

		tmpL = rs232_handle->rxFrame.dataPtr[2];
		tmpH = rs232_handle->rxFrame.dataPtr[3];
		zitai[1] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f * 180.0f; // Y

		tmpL = rs232_handle->rxFrame.dataPtr[4];
		tmpH = rs232_handle->rxFrame.dataPtr[5];
		zitai[2] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f * 180.0f; // Z

		for (int i = 0; i < 3; i++)
			imuHandle.zitai[i] = zitai[i];
	}
}

static void rs232_data_siyuanshu(void)
{
	float siyuanshu[4];
	uint8_t tmpH, tmpL;

	if (rs232_handle->rxFrame.dataLen == 8)
	{
		tmpL = rs232_handle->rxFrame.dataPtr[0];
		tmpH = rs232_handle->rxFrame.dataPtr[1];
		siyuanshu[0] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f;

		tmpL = rs232_handle->rxFrame.dataPtr[2];
		tmpH = rs232_handle->rxFrame.dataPtr[3];
		siyuanshu[1] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f;

		tmpL = rs232_handle->rxFrame.dataPtr[4];
		tmpH = rs232_handle->rxFrame.dataPtr[5];
		siyuanshu[2] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f;

		tmpL = rs232_handle->rxFrame.dataPtr[6];
		tmpH = rs232_handle->rxFrame.dataPtr[7];
		siyuanshu[3] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f;

		for (int i = 0; i < 4; i++)
			imuHandle.siyuanshu[i] = siyuanshu[i];
	}
}

static void rs232_data_gyro_acc(void)
{
	float gyro[3], acc[3];
	uint8_t tmpH, tmpL;

	if (rs232_handle->rxFrame.dataLen == 12)
	{
		for (int i = 0; i < 3; i++)
		{
			tmpL = rs232_handle->rxFrame.dataPtr[i * 2];
			tmpH = rs232_handle->rxFrame.dataPtr[i * 2 + 1];
			acc[i] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f * IMU_ACC_RANGE;
		}

		for (int i = 0; i < 3; i++)
		{
			tmpL = rs232_handle->rxFrame.dataPtr[i * 2 + 6];
			tmpH = rs232_handle->rxFrame.dataPtr[i * 2 + 7];
			gyro[i] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f * IMU_GYPO_RANGE;
		}

		for (int i = 0; i < 3; i++)
		{
			imuHandle.acc[i] = acc[i];
			imuHandle.gyro[i] = gyro[i];
		}
	}
}

static void rs232_data_magnet(void)
{
	float magnet[3];
	float temperature;
	uint8_t tmpH, tmpL;

	if (rs232_handle->rxFrame.dataLen == 8)
	{
		for (int i = 0; i < 3; i++)
		{
			tmpL = rs232_handle->rxFrame.dataPtr[i * 2];
			tmpH = rs232_handle->rxFrame.dataPtr[i * 2 + 1];
			magnet[i] = (float)((int16_t)(tmpH << 8) | tmpL);
		}

		tmpL = rs232_handle->rxFrame.dataPtr[6];
		tmpH = rs232_handle->rxFrame.dataPtr[7];
		temperature = (float)((int16_t)(tmpH << 8) | tmpL) / 100.0f;

		for (int i = 0; i < 3; i++)
		{
			imuHandle.magnet[i] = magnet[i];
		}
		imuHandle.temperature = temperature * 0.01f + imuHandle.temperature * 0.99f;
	}
}

static void rs232_data_pressure(void)
{
	float pressure;
	float altitude;
	float temperature;
	uint8_t tmpH, tmpL;
	int itmp;

	if (rs232_handle->rxFrame.dataLen == 10)
	{
		tmpL = rs232_handle->rxFrame.dataPtr[0];
		tmpH = rs232_handle->rxFrame.dataPtr[1];
		itmp = ((int)(tmpH << 8) | tmpL);
		tmpL = rs232_handle->rxFrame.dataPtr[2];
		tmpH = rs232_handle->rxFrame.dataPtr[3];
		pressure = ((((int)(tmpH << 8) | tmpL) << 16) + itmp);

		tmpL = rs232_handle->rxFrame.dataPtr[4];
		tmpH = rs232_handle->rxFrame.dataPtr[5];
		itmp = ((int)(tmpH << 8) | tmpL);
		tmpL = rs232_handle->rxFrame.dataPtr[6];
		tmpH = rs232_handle->rxFrame.dataPtr[7];
		altitude = ((((int)(tmpH << 8) | tmpL) << 16) + itmp);

		tmpL = rs232_handle->rxFrame.dataPtr[8];
		tmpH = rs232_handle->rxFrame.dataPtr[9];
		temperature = (float)((int16_t)(tmpH << 8) | tmpL) / 100.0f;

		imuHandle.pressure = pressure;
		imuHandle.altitude = altitude * 1e-3f;
		imuHandle.temperature = temperature * 0.01f + imuHandle.temperature * 0.99f;
	}
}

/*
static void imu_tuoluoyisiyuanshu_revc(void)
{
	uint8_t tmpH, tmpL;
	uint8_t i;
	uint8_t *data = rs232_handle->rxFrame.dataPtr;
	float accel[6];
	const float jiasuduK = 4.0f / 32768.0f;
	const float tuoluoyiK = 2000.0f / 32768.0f;

	if (rs232_handle->rxFrame.dataLen == 12)
	{
		i = 0;
		tmpL = data[i++];
		tmpH = data[i++];

		accel[0] = (float)((int16_t)((int16_t)(tmpH << 8) | tmpL)) * jiasuduK;

		tmpL = data[i++];
		tmpH = data[i++];
		accel[1] = (float)((int16_t)((int16_t)(tmpH << 8) | tmpL)) * jiasuduK;

		tmpL = data[i++];
		tmpH = data[i++];
		accel[2] = (float)((int16_t)((int16_t)(tmpH << 8) | tmpL)) * jiasuduK;

		tmpL = data[i++];
		tmpH = data[i++];
		accel[3] = (float)((int16_t)((int16_t)(tmpH << 8) | tmpL)) * tuoluoyiK;

		tmpL = data[i++];
		tmpH = data[i++];
		accel[4] = (float)((int16_t)((int16_t)(tmpH << 8) | tmpL)) * tuoluoyiK;

		tmpL = data[i++];
		tmpH = data[i++];
		accel[5] = (float)((int16_t)((int16_t)(tmpH << 8) | tmpL)) * tuoluoyiK;

		fAccelRecieved(accel);
	}
} // */

/** 数据发送处理函数群
**
**/
static void imu_uart_send(void)
{
	// if ( (rs232_handle->huart->hdmatx->Instance->NDTR == 0) || ((rs232_handle->huart->hdmatx->Instance->CR & DMA_SxCR_EN_Msk) == 0x0) )
	if (rs232_handle->huart->hdmatx->Instance->NDTR == 0)
	{
		uint32_t len = imu_uart_frame_construct();

		// imu_senddata_send_to_computer(len);
		uart_tx_dma_start(rs232_handle->huart, len);
	}
}

static uint32_t imu_uart_frame_construct(void)
{
	uint8_t *txbuf = rs232_handle->huart->pTxBuffPtr;
	uint8_t imu_data_len = uart232_handle.txDataBuffer[1] + 2;

	// 桢头
	uint8_t i;
	txbuf[0] = 0x55;
	txbuf[1] = 0xAF;
	// for(i=0; i<rs232_handle->frame.headerLen; i++)
	//{
	//	txbuf[i] = rs232_handle->frame.header[i];
	// }

	// 数据 （命令+长度+数据）
	uint8_t *txdatabuf = &txbuf[rs232_handle->txFrame.headerLen];
	for (i = 0; i < imu_data_len; i++)
	{
		txdatabuf[i] = uart232_handle.txDataBuffer[i];
	}

	// 校验位
	uint32_t tmp = 0;
	for (i = 0; i < rs232_handle->txFrame.headerLen; i++)
	{
		tmp += txbuf[i];
	}

	for (i = 0; i < imu_data_len; i++)
	{
		tmp += uart232_handle.txDataBuffer[i];
	}

	txbuf[rs232_handle->txFrame.headerLen + imu_data_len] = (uint8_t)tmp; // 校验位的低字节
	// txbuf[rs232_handle->txFrame.headerLen + imu_data_len + 1] = (uint8_t)(tmp>>8); // 校验位的高字节

	return (rs232_handle->txFrame.headerLen + imu_data_len + rs232_handle->txFrame.checkLen); // 返回要发送的字节个数
}

void imu_baud_set(uint8_t baudrate) // 设为115200
{
	uart232_handle.txDataBuffer[0] = IMUCMD_BAUD & 0x7F; // bit7=0代表写
	uart232_handle.txDataBuffer[1] = 1;					 // 数据长度为1
	uart232_handle.txDataBuffer[2] = baudrate;			 // baudrate=4: 115200

	imu_uart_send();
}

// 回传速率设置
// [0-9]
// 1: 200Hz, 6:10Hz, 9:1Hz, 4:50Hz, 5:20Hz, 3:100Hz, 2:125Hz
void imu_returnrate_set(uint8_t returnrate) // 设置回传速度
{
	uart232_handle.txDataBuffer[0] = IMUCMD_RETURNRATE & 0x7F; // bit7=0代表写
	uart232_handle.txDataBuffer[1] = 1;						   // 数据长度为1
	uart232_handle.txDataBuffer[2] = returnrate;			   // returnrate=1: 200Hz, 6:10Hz, 9:1Hz, 4:50Hz, 5:20Hz, 3:100Hz, 2:125Hz

	imu_uart_send();
}

// 回传内容设置 0不上传;  1上传
// bit 0: 姿态
// bit 1: 四元数
// bit 2: 陀螺仪和加速度
// bit 3: 磁力计
// bit 4: 气压计
// bit 5: 端口状态
// bit 6: 数据匿名
// bit 7: reserved to 0
void imu_returncontext_set(uint8_t returncontext) // 设置回传内容
{
	uart232_handle.txDataBuffer[0] = IMUCMD_RETURNSET & 0x7F; // bit7=0代表写
	uart232_handle.txDataBuffer[1] = 1;						  // 数据长度为1
	uart232_handle.txDataBuffer[2] = returncontext;			  // bit0:角度, bit2:陀螺仪和加速度

	imu_uart_send();
}

void imu_alg_set(uint8_t alg) // 设置算法 9轴 or 6轴
{
	uart232_handle.txDataBuffer[0] = IMUCMD_ALG & 0x7F; // bit7=0代表写
	uart232_handle.txDataBuffer[1] = 1;					// 数据长度为1
	uart232_handle.txDataBuffer[2] = alg;				// 0:六轴解算，1：九轴解算

	imu_uart_send();
}

void imu_save(void) // 保存设置
{
	uart232_handle.txDataBuffer[0] = IMUCMD_SAVE & 0x7F; // bit7=0代表写
	uart232_handle.txDataBuffer[1] = 1;					 // 数据长度为1
	uart232_handle.txDataBuffer[2] = 0;					 //

	imu_uart_send();
}

// range [0, 3]
// 0: 250dps
// 1: 500dps
// 2: 1000dps
// 3: 2000dps (default)
void imu_gyro_range_set(int range)
{
	if (range > 3)
		return;

	uart232_handle.txDataBuffer[0] = IMUCMD_GYRO_RANGE & 0x7F; // 写的话bit7要置0
	uart232_handle.txDataBuffer[1] = 1;						   // 数据长度为1
	uart232_handle.txDataBuffer[2] = range;					   //

	imu_uart_send();
}

// range [0, 3], unit G = 9.8m/s^2
// 0: 2G
// 1: 4G (default)
// 2: 8G
// 3: 16G
void imu_acc_range_set(int range)
{
	if (range > 3)
		return;

	uart232_handle.txDataBuffer[0] = IMUCMD_ACC_RANGE & 0x7F; // 写的话bit7要置0
	uart232_handle.txDataBuffer[1] = 1;						  // 数据长度为1
	uart232_handle.txDataBuffer[2] = range;					  //

	imu_uart_send();
}

void imu_led_set(bool on)
{
	uart232_handle.txDataBuffer[0] = 0x0F & 0x7F; // 写的话bit7要置0
	uart232_handle.txDataBuffer[1] = 1;			  // 数据长度为1
	uart232_handle.txDataBuffer[2] = on ? 0 : 1;  //

	imu_uart_send();
}
