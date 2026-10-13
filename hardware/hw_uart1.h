/*
 * hw_uart1.h - USART1 (PA9 TX / PA10 RX) used as the calibration console AND
 * the JDY-23 BLE transparent channel.
 *
 * 9600 8N1 matches the JDY-23 default, so the same wires serve both a plain
 * USB-TTL (calibration wizard, tools/cal_host.py) and the BLE module
 * (1 Hz measurement report / remote console from a phone).
 */

#ifndef HW_UART1_H_
#define HW_UART1_H_

#include <stdint.h>

#define CAL_UART_BAUD   9600U

void     HW_UART1_Init(void);
void     HW_UART1_Send(const char *s);
void     HW_UART1_Send_Block(const uint8_t *d, uint16_t n);
uint8_t  HW_UART1_GetByte(uint8_t *b);   /* returns 1 when one byte arrived */

#endif /* HW_UART1_H_ */
