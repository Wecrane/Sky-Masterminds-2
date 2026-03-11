#include "Uart_Config.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

#ifndef USART2_RX_BUFFER_SIZE
#define USART2_RX_BUFFER_SIZE 256
#endif

static volatile uint8_t s_usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
static volatile uint16_t s_usart2_rx_head = 0;
static volatile uint16_t s_usart2_rx_tail = 0;

static void Uart2_PutChar(uint8_t ch)
{
	volatile uint32_t timeout = 50000;
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
	{
		if(--timeout == 0) return; // 超时保护
	}
	USART_SendData(USART2, ch);
}

void Uart2_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;

	if(baudrate == 0) baudrate = 115200;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// PA2: TX 复用推挽
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio);

	// PA3: RX 浮空输入
	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio);

	// USART2 参数
	USART_StructInit(&usart);
	usart.USART_BaudRate = baudrate;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &usart);

	// 使能接收中断，环形缓冲
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	// NVIC 配置
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	USART_Cmd(USART2, ENABLE);
}

extern uint16_t uart_rev_tiem;
void USART2_IRQHandler(void)
{
	// ORE溢出错误处理：必须先读SR再读DR来清除，否则RXNE将停止工作
	if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
	{
		(void)USART_ReceiveData(USART2); // 读DR清除ORE标志
	}

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		uart_rev_tiem = 0;
		uint8_t data = (uint8_t)USART_ReceiveData(USART2);
		uint16_t next = (uint16_t)((s_usart2_rx_head + 1) % USART2_RX_BUFFER_SIZE);
		if(next != s_usart2_rx_tail)
		{
			s_usart2_rx_buffer[s_usart2_rx_head] = data;
			s_usart2_rx_head = next;
		}
		// 否则丢弃字节以避免覆盖
	}
}

void Uart2_SendByte(uint8_t byte)
{
	volatile uint32_t timeout = 50000;
	Uart2_PutChar(byte);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{
		if(--timeout == 0) return;
	}
}

void Uart2_SendBuf(const uint8_t *buf, uint16_t len)
{
	volatile uint32_t timeout;
	if(buf == 0 || len == 0) return;
	while(len--)
	{
		Uart2_PutChar(*buf++);
	}
	timeout = 50000;
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{
		if(--timeout == 0) return;
	}
}

void Uart2_SendString(const char *str)
{
	volatile uint32_t timeout;
	if(!str) return;
	while(*str)
	{
		Uart2_PutChar((uint8_t)*str++);
	}
	timeout = 50000;
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{
		if(--timeout == 0) return;
	}
}

// 非阻塞读取一个字节，如果没有数据返回 0xFF 并设置 *valid = 0
uint8_t Uart2_ReadByte(uint8_t *valid)
{
	uint16_t head, tail;
	// 临界区保护
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	head = s_usart2_rx_head;
	tail = s_usart2_rx_tail;
	
	if(head == tail)
	{
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		if(valid) *valid = 0;
		return 0xFF;
	}
	
	uint8_t ch = s_usart2_rx_buffer[tail];
	s_usart2_rx_tail = (uint16_t)((tail + 1) % USART2_RX_BUFFER_SIZE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	if(valid) *valid = 1;
	return ch;
}

// 保留旧接口以兼容（添加超时保护和临界区）
uint8_t Uart2_ReadByteBlocking(void)
{
	volatile uint32_t timeout = 100000; // 超时保护
	while(1)
	{
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
		if(s_usart2_rx_head != s_usart2_rx_tail)
		{
			uint8_t ch = s_usart2_rx_buffer[s_usart2_rx_tail];
			s_usart2_rx_tail = (uint16_t)((s_usart2_rx_tail + 1) % USART2_RX_BUFFER_SIZE);
			USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
			return ch;
		}
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		if(--timeout == 0) return 0xFF; // 超时返回
	}
}

int Uart2_BytesAvailable(void)
{
	// 临界区保护：读取 head/tail 时禁止 USART 中断，避免竞态条件
	uint16_t head, tail;
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	head = s_usart2_rx_head;
	tail = s_usart2_rx_tail;
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	if(head >= tail)
		return (int)(head - tail);
	else
		return (int)(USART2_RX_BUFFER_SIZE - (tail - head));
}

// printf 重定向
int fputc(int ch, FILE *f)
{
	(void)f;
	Uart2_PutChar((uint8_t)ch);
	return ch;
}


