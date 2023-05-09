#ifndef UART_IO_H 
#define UART_IO_H
#include "esp_err.h"

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024) // Kích thước của bộ đệm RingBuffer
#define RD_BUF_SIZE (BUF_SIZE)


typedef void (*uart_handler_t) (uint8_t *data, uint16_t size);

void uart_init(void);
void uart_push(uint8_t *data, uint16_t size);
void uart_set_callback(void * cb);

#endif