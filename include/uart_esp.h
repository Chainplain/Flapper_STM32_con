#ifndef uart_esp_h
#define uart_esp_h

#include <stm32f4xx_hal.h>
#include "communication/uart_232.h"
#include "cmsis_os.h"

UART232_Handle *rs232_esp_init(void);

uint8_t uart_esp_send(uint8_t *data, int length);
uint8_t uart_esp_send_str(char *sbuf);

extern osThreadId esp_uart_thread_id;
extern volatile int EspFlag;

#endif