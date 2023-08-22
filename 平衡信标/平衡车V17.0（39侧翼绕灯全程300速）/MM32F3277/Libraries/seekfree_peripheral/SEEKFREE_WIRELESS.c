/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2018,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				SEEKFREE_WIRELESS
* @company			�ɶ���ɿƼ����޹�˾
* @author			��ɿƼ�(QQ3184284598)
* @version			�鿴doc��version�ļ� �汾˵��
* @Software			IAR 8.3 or MDK 5.24
* @Taobao			https://seekfree.taobao.com/
* @date				2020-11-23
* @note
* 					���߶��壺
* 					------------------------------------
* 					ģ��ܽ�				��Ƭ���ܽ�
* 					RX						�鿴SEEKFREE_WIRELESS.h�ļ��е�WIRELESS_UART_TX�궨��
* 					TX						�鿴SEEKFREE_WIRELESS.h�ļ��е�WIRELESS_UART_RX�궨��
* 					RTS						�鿴SEEKFREE_WIRELESS.h�ļ��е�RTS_PIN�궨��
* 					CMD						���ջ�������
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
// @brief		����ת����ģ�� �����жϺ���
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
// @note		�ú�����ISR�ļ� �����жϳ��򱻵���
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
// @brief		����ת����ģ���ʼ��
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
// @note
//-------------------------------------------------------------------------------------------------------------------
void seekfree_wireless_init (void)
{
	
	//������ʹ�õĲ�����Ϊ460800��Ϊ����ת����ģ���Ĭ�ϲ����ʣ�������������������������ģ�鲢�޸Ĵ��ڵĲ�����
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
	
	// ��������λ����  ָʾ��ǰģ���Ƿ���Խ�������  0���Լ�������  1�����Լ�������
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
// @brief		����ת����ģ�� ���ͺ���
// @param		buff			��Ҫ���͵����ݵ�ַ
// @param		len				���ͳ���
// @return		uint32			ʣ��δ���͵��ֽ���
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
			return len;																	// ģ��æ,�������ǰ����ʹ��while�ȴ� �����ʹ�ú���ע�͵�while�ȴ�����滻��if���
		}
		//while(gpio_get(RTS_PIN));														// ���RTSΪ�͵�ƽ���������������
		uart4_putbuff(buff,30);

		buff += 30;																		// ��ַƫ��
		len -= 30;																		// ����
	}

	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12))
	{
	return len;																			// ģ��æ,�������ǰ����ʹ��while�ȴ� �����ʹ�ú���ע�͵�while�ȴ�����滻��if���
	}
	//while(gpio_get(RTS_PIN));															// ���RTSΪ�͵�ƽ���������������
	uart4_putbuff(buff,len);												// ������������

	return 0;
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		���ڷ�������
// @param		uartn			����ģ���(USART_1,USART_2)
// @param		*buff			Ҫ���͵������ַ
// @param		len				���ͳ���
// @return		void
// Sample usage:				uart_putbuff(USART_1,&a[0],5);
//-------------------------------------------------------------------------------------------------------------------
void uart4_putbuff(uint8 *buff, uint32 len)
{
	while(len)																					// ѭ����������
	{
		UART4->TDR = *buff++;														// д�뷢������
		while(!(UART4->CSR & UART_CSR_TXC));										// �ȴ��������
		len--;
	}
}
void uart4_rx_irq(void)
{
	UART4->IER |= UART_IER_RX;
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;                                  //�жϺ�����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0x00;	//��ռ���ȼ�ֵԽС�����ȼ�Խ��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0x00;				//��ռ���ȼ�ֵԽС�����ȼ�Խ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE ;								//ʹ��
	NVIC_Init(&NVIC_InitStructure);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		���ߴ���DMA
// @param		uartn			����ģ���(USART_1,USART_2)
// @param		*buff			Ҫ���͵������ַ
// @param		len				���ͳ���
// @return		void
// Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void wireless_usart_dma(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA2, ENABLE);

		//MDA���ó�ʼ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->TDR;										// �����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)camera_buffer_addr;											// �ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;											// �ڴ���ΪԴ
	DMA_InitStructure.DMA_BufferSize = MT9V03X_H*MT9V03X_W+4;										// ������ٸ�����
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;							// �����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;										// �ڴ��ַ����+1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;						// ����ÿ�δ���һ���ֽ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;								// �ڴ�ÿ�δ���һ���ֽ�
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;												// ��ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_CCR_PL_High;										// ���ȼ���
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;												// ���ڴ浽�ڴ�ģʽ
	DMA_InitStructure.DMA_Auto_reload = DMA_Auto_Reload_Disable;
	DMA_Init(DMA2_Channel5, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);													// ����DMA��������ж�

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel5_IRQn;                                  //�жϺ�����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0x01;	//��ռ���ȼ�ֵԽС�����ȼ�Խ��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0x01;				//��ռ���ȼ�ֵԽС�����ȼ�Խ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ��
	NVIC_Init(&NVIC_InitStructure);
	
	UART_DMACmd(UART4 , UART_GCR_DMA , ENABLE);//UART_GCR_DMA UART_DMAReq_EN
	DMA_Cmd(DMA2_Channel5, ENABLE);																	// ����DMA1
}
//dma��������ж�
void wireless_usart_dma_interrupt(void)
{
	uart1_putchar(UART4,0x00);
	uart1_putchar(UART4,0xff);
	uart1_putchar(UART4,0x01);
	uart1_putchar(UART4,0x01);	
	wireless_usart_dma();
}
//���ߴ���dma���俪��
void open_usart4_dma(void)
{
	wireless_usart_dma();
}
void close_usart4_dma(void)
{
	DMA_Cmd(DMA2_Channel5, DISABLE);	
}