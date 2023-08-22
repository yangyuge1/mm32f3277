/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,逐飞科技
* All rights reserved.
* 技术讨论QQ群：一群：179029047(已满)  二群：244861897
*
* 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
*
* @file				board
* @company			成都逐飞科技有限公司
* @author			逐飞科技(QQ3184284598)
* @version			查看doc内version文件 版本说明
* @Software			IAR 8.3 or MDK 5.24
* @Target core		MM32F3277
* @Taobao			https://seekfree.taobao.com/
* @date				2021-02-22
********************************************************************************************************************/

#include "board.h"
//#include "zf_uart.h"
#include "hal_rcc.h"
#include "hal_gpio.h"
#include "hal_uart.h"
#include "hal_misc.h"
//-------------------------------------------------------------------------------------------------------------------
// @brief		核心板初始化
// @param		debug_enable	是否开启默认 debug 输出 默认 UART1 
// @return		void
// Sample usage:				board_init(TRUE);
//-------------------------------------------------------------------------------------------------------------------
void board_init (void)
{

//		uart_init(DEBUG_UART, DEBUG_UART_BAUD, DEBUG_UART_TX, DEBUG_UART_RX);						// 默认初始化 UART1 用以支持 printf 输出
	
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

	//UART1_TX   GPIOA.9
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//UART1_RX    GPIOA.10
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	UART_InitTypeDef UART_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_UART1, ENABLE);
	//Baud rate
	UART_StructInit(&UART_InitStruct);
	UART_InitStruct.BaudRate = 115200;
//	UART_InitStruct.BaudRate = 460800;
	//The word length is in 8-bit data format.
	UART_InitStruct.WordLength = UART_WordLength_8b;
	UART_InitStruct.StopBits = UART_StopBits_1;
	//No even check bit.
	UART_InitStruct.Parity = UART_Parity_No;
	//No hardware data flow control.
	UART_InitStruct.HWFlowControl = UART_HWFlowControl_None;
	UART_InitStruct.Mode = UART_Mode_Rx | UART_Mode_Tx;

	UART_Init(UART1, &UART_InitStruct);
//    UART_ITConfig(UART1, UART_IT_TXIEN|UART_IT_RXIEN, ENABLE);
	UART_Cmd(UART1, ENABLE);
	
}
void uart1_rx_irq(void)
{
	UART1->IER |= UART_IER_RX;
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQn;                                  //中断号设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0x00;	//抢占优先级值越小，优先级越高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0x00;				//抢占优先级值越小，优先级越高
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE ;								//使能
	NVIC_Init(&NVIC_InitStructure);
}