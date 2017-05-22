#ifndef UART_H
#define UART_H

#include <stdint.h>

// busy-loop print zero-terminated C string.
void uart_print_string_blocking(const char *buf);

// busy-loop send binary uint8 buffer
void uart_send_blocking(const uint8_t *buf, int len);

// Handle a message, if there is any. Function returns quickly if there is no message.
// You can call this at 1 kHz.
void handle_uart_message();

// The uart ISR, should be of high priority to data register overruns (there is no DMA for UART in the lousy STM32!)
void uart_rx_handler();



#endif
