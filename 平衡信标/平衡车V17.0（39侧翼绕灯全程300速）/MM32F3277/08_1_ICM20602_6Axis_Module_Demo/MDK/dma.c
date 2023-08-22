#include "dma.h"
#include "hal_rcc.h"
#include "hal_dma.h"
#include "common.h"
void wireless_dma_init(DMA_Channel_TypeDef* dma_ch, uint32 src_addr, uint32 des_addr, uint32 size)
{
	DMA_InitTypeDef DMA_InitStructure;
	//MDA配置初始化
	DMA_InitStructure.DMA_PeripheralBaseAddr = src_addr;										// 源地址
	DMA_InitStructure.DMA_MemoryBaseAddr = des_addr;											// 目标地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;											// 外设作为源
	DMA_InitStructure.DMA_BufferSize = size;													// 传输多少个数据
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;							// 外设地址不增加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;										// 内存地址依次+1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;						// 外设每次传输一个字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;								// 内存每次传输一个字节
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												// 循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;										// 优先级最高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;												// 非内存到内存模式
	DMA_InitStructure.DMA_Auto_reload = DMA_Auto_Reload_Enable;
	
	DMA_Init(dma_ch, &DMA_InitStructure);

	DMA_ITConfig(dma_ch, DMA_IT_TC, ENABLE);													// 配置DMA传输完成中断
	DMA_Cmd(dma_ch, ENABLE);																	// 开启DMA1
}

extern u32 UART4_RX_BUF;
void uart_dma_init(void)
{
	//DMA2总线初始化
	RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA2, ENABLE);
	//DMA串口初始化
	wireless_dma_init(DMA2_Channel3, (uint32)&uart_index[WIRELESS_UART]->RDR, (uint32)UART4_RX_BUF,16);
	//中断配置
	nvic_init(DMA2_Channel3_IRQn, 0x00, 0x01, ENABLE);
}
