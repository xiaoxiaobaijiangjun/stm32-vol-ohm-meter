/*
 * hw_uart1.c - polled USART1 driver (no interrupts, matching the rest of the
 * application). Pins PA9/PA10 are free on this board (see the .ioc pin list).
 */

#include "hw_uart1.h"
#include "stm32f1xx_hal.h"

static UART_HandleTypeDef hcal_uart;

void HW_UART1_Init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	/* F1 style: TX = AF push-pull, RX = floating input */
	gpio.Pin = GPIO_PIN_9;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Pin = GPIO_PIN_10;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpio);

	hcal_uart.Instance = USART1;
	hcal_uart.Init.BaudRate = CAL_UART_BAUD;
	hcal_uart.Init.WordLength = UART_WORDLENGTH_8B;
	hcal_uart.Init.StopBits = UART_STOPBITS_1;
	hcal_uart.Init.Parity = UART_PARITY_NONE;
	hcal_uart.Init.Mode = UART_MODE_TX_RX;
	hcal_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hcal_uart.Init.OverSampling = UART_OVERSAMPLING_16;
	(void)HAL_UART_Init(&hcal_uart);
}

void HW_UART1_Send_Block(const uint8_t *d, uint16_t n)
{
	(void)HAL_UART_Transmit(&hcal_uart, (uint8_t *)d, n, 200U);
}

void HW_UART1_Send(const char *s)
{
	uint16_t n = 0U;

	while(s[n] != '\0')
	{
		n++;
	}
	HW_UART1_Send_Block((const uint8_t *)s, n);
}

uint8_t HW_UART1_GetByte(uint8_t *b)
{
	return (HAL_UART_Receive(&hcal_uart, b, 1U, 0U) == HAL_OK) ? 1U : 0U;
}
