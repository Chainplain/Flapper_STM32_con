#ifndef interface_uart_h
#define interface_uart_h

#include <stm32f4xx_hal.h>
#include "uart_232.h"
#include "commDataAnalysis.h"
#include "cmsis_os.h"

UART232_Handle *rs232_computer_init(void);

class UartComm : public CommDataAnalysis
{
public:
    UartComm();

public:
    virtual uint8_t send(uint8_t *buf, int length);
	virtual uint8_t send(char *str);
	virtual uint8_t send(uint8_t *cmd, uint8_t *data, uint32_t datalength);
    virtual uint8_t send(char cmd, uint8_t *data, int datalength);
};

extern osThreadId sendUart_thread_id;

#endif