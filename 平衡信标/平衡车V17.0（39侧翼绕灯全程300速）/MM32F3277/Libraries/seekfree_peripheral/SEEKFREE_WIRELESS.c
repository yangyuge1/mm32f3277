/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2018,逐飞科技
* All rights reserved.
* 技术讨论QQ群：一群：179029047(已满)  二群：244861897
*
* 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
*
* @file				SEEKFREE_WIRELESS
* @company			成都逐飞科技有限公司
* @author			逐飞科技(QQ3184284598)
* @version			查看doc内version文件 版本说明
* @Software			IAR 8.3 or MDK 5.24
* @Taobao			https://seekfree.taobao.com/
* @date				2020-11-23
* @note
* 					接线定义：
* 					------------------------------------
* 					模块管脚				单片机管脚
* 					RX						查看SEEKFREE_WIRELESS.h文件中的WIRELESS_UART_TX宏定义
* 					TX						查看SEEKFREE_WIRELESS.h文件中的WIRELESS_UART_RX宏定义
* 					RTS						查看SEEKFREE_WIRELESS.h文件中的RTS_PIN宏定义
* 					CMD						悬空或者上拉
* 					------------------------------------
********************************************************************************************************************/

#include "zf_systick.h"
#include "hal_rcc.h"
#include "hal_gpio.h"
#include "hal_uart.h"
#include "hal_dma.h"
#include "SEEKFREE_WIRELESS.h"
#include "SEEKFREE_MT9V03X.h"
#include "string.h"
#include "hal_misc.h"
//uint8 wireless_rx_buffer[WIRELESS_BUFFER_SIZE];
//uint16 wireless_rx_index = 0;
//extern float recive;
//-------------------------------------------------------------------------------------------------------------------
// @brief		无线转串口模块 串口中断函数
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
// @note		该函数在ISR文件 串口中断程序被调用
//-------------------------------------------------------------------------------------------------------------------
//void wireless_uart_callback()
//{
//	wireless_rx_buffer[wireless_rx_index++] = (uart_index[WIRELESS_UART])->RDR & 0x00FF;	
//	if(wireless_rx_index==WIRELESS_BUFFER_SIZE)
//	{
//		wireless_rx_index = 0;
//	}
//}

//-------------------------------------------------------------------------------------------------------------------
// @brief		无线转串口模块初始化
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
// @note
//-------------------------------------------------------------------------------------------------------------------
void seekfree_wireless_init (void)
{
	
	//本函数使用的波特率为460800，为无线转串口模块的默认波特率，如需其他波特率请自行配置模块并修改串口的波特率
	GPIO_InitTypeDef GPIO_InitStruct;
	UART_InitTypeDef UART_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_UART4, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOC, ENABLE);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_11);
	//UART4_TX   GPIOC.10
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	//UART4_RX    GPIOC.11
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	// 定义流控位引脚  指示当前模块是否可以接受数据  0可以继续接收  1不可以继续接收
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	

	
	//Baud rate
	UART_StructInit(&UART_InitStruct);
	UART_InitStruct.BaudRate = 460800;
	//The word length is in 8-bit data format.
	UART_InitStruct.WordLength = UART_WordLength_8b;
	UART_InitStruct.StopBits = UART_StopBits_1;
	//No even check bit.
	UART_InitStruct.Parity = UART_Parity_No;
	//No hardware data flow control.
	UART_InitStruct.HWFlowControl = UART_HWFlowControl_None;
	UART_InitStruct.Mode = UART_Mode_Rx | UART_Mode_Tx;

	UART_Init(UART4, &UART_InitStruct);
	//UART_ITConfig(UART1, UART_IT_TXIEN|UART_IT_RXIEN, ENABLE);
	UART_Cmd(UART4, ENABLE);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		无线转串口模块 发送函数
// @param		buff			需要发送的数据地址
// @param		len				发送长度
// @return		uint32			剩余未发送的字节数
// @since		v1.0
// Sample usage:
// @note
//-------------------------------------------------------------------------------------------------------------------
uint32 seekfree_wireless_send_buff(uint8 *buff, uint32 len)
{
	while(len>30)
	{
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12))
		{
			return len;																	// 模块忙,如果允许当前程序使用while等待 则可以使用后面注释的while等待语句替换本if语句
		}
		//while(gpio_get(RTS_PIN));														// 如果RTS为低电平，则继续发送数据
		uart4_putbuff(buff,30);

		buff += 30;																		// 地址偏移
		len -= 30;																		// 数量
	}

	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12))
	{
	return len;																			// 模块忙,如果允许当前程序使用while等待 则可以使用后面注释的while等待语句替换本if语句
	}
	//while(gpio_get(RTS_PIN));															// 如果RTS为低电平，则继续发送数据
	uart4_putbuff(buff,len);												// 发送最后的数据

	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		串口发送数组
// @param		uartn			串口模块号(USART_1,USART_2)
// @param		*buff			要发送的数组地址
// @param		len				发送长度
// @return		void
// Sample usage:				uart_putbuff(USART_1,&a[0],5);
//-------------------------------------------------------------------------------------------------------------------
void uart4_putbuff(uint8 *buff, uint32 len)
{
	while(len)																					// 循环到发送完
	{
		UART4->TDR = *buff++;														// 写入发送数据
		while(!(UART4->CSR & UART_CSR_TXC));										// 等待发送完成
		len--;
	}
}
void uart4_rx_irq(void)
{
	UART4->IER |= UART_IER_RX;
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;                                  //中断号设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0x00;	//抢占优先级值越小，优先级越高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0x00;				//抢占优先级值越小，优先级越高
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE ;								//使能
	NVIC_Init(&NVIC_InitStructure);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		无线串口DMA
// @param		uartn			串口模块号(USART_1,USART_2)
// @param		*buff			要发送的数组地址
// @param		len				发送长度
// @return		void
// Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void wireless_usart_dma(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA2, ENABLE);

		//MDA配置初始化
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->TDR;										// 外设地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)camera_buffer_addr;											// 内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;											// 内存作为源
	DMA_InitStructure.DMA_BufferSize = MT9V03X_H*MT9V03X_W+4;										// 传输多少个数据
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;							// 外设地址不增加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;										// 内存地址依次+1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;						// 外设每次传输一个字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;								// 内存每次传输一个字节
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;												// 非循环模式
	DMA_InitStructure.DMA_Priority = DMA_CCR_PL_High;										// 优先级高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;												// 非内存到内存模式
	DMA_InitStructure.DMA_Auto_reload = DMA_Auto_Reload_Disable;
	DMA_Init(DMA2_Channel5, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);													// 配置DMA传输完成中断

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel5_IRQn;                                  //中断号设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0x01;	//抢占优先级值越小，优先级越高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0x01;				//抢占优先级值越小，优先级越高
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能
	NVIC_Init(&NVIC_InitStructure);
	
	UART_DMACmd(UART4 , UART_GCR_DMA , ENABLE);//UART_GCR_DMA UART_DMAReq_EN
	DMA_Cmd(DMA2_Channel5, ENABLE);																	// 开启DMA1
}
//dma传输完成中断
void wireless_usart_dma_interrupt(void)
{
	uart1_putchar(UART4,0x00);
	uart1_putchar(UART4,0xff);
	uart1_putchar(UART4,0x01);
	uart1_putchar(UART4,0x01);	
	wireless_usart_dma();
}
//无线串口dma传输开启
void open_usart4_dma(void)
{
	wireless_usart_dma();
}
void close_usart4_dma(void)
{
	DMA_Cmd(DMA2_Channel5, DISABLE);	
}