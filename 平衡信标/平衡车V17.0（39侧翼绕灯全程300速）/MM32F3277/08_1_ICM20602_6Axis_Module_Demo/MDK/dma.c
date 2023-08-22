#include "dma.h"
#include "hal_rcc.h"
#include "hal_dma.h"
#include "common.h"
void wireless_dma_init(DMA_Channel_TypeDef* dma_ch, uint32 src_addr, uint32 des_addr, uint32 size)
{
	DMA_InitTypeDef DMA_InitStructure;
	//MDA���ó�ʼ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = src_addr;										// Դ��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = des_addr;											// Ŀ���ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;											// ������ΪԴ
	DMA_InitStructure.DMA_BufferSize = size;													// ������ٸ�����
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;							// �����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;										// �ڴ��ַ����+1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;						// ����ÿ�δ���һ���ֽ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;								// �ڴ�ÿ�δ���һ���ֽ�
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												// ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;										// ���ȼ����
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;												// ���ڴ浽�ڴ�ģʽ
	DMA_InitStructure.DMA_Auto_reload = DMA_Auto_Reload_Enable;
	
	DMA_Init(dma_ch, &DMA_InitStructure);

	DMA_ITConfig(dma_ch, DMA_IT_TC, ENABLE);													// ����DMA��������ж�
	DMA_Cmd(dma_ch, ENABLE);																	// ����DMA1
}

extern u32 UART4_RX_BUF;
void uart_dma_init(void)
{
	//DMA2���߳�ʼ��
	RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA2, ENABLE);
	//DMA���ڳ�ʼ��
	wireless_dma_init(DMA2_Channel3, (uint32)&uart_index[WIRELESS_UART]->RDR, (uint32)UART4_RX_BUF,16);
	//�ж�����
	nvic_init(DMA2_Channel3_IRQn, 0x00, 0x01, ENABLE);
}
