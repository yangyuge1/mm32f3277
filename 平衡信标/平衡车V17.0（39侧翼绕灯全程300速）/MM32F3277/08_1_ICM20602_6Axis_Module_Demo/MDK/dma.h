#ifndef _dma_h
#define _dma_h

#include "board.h"
#include "headfile.h"
void wireless_dma_init(DMA_Channel_TypeDef* dma_ch, uint32 src_addr, uint32 des_addr, uint32 size);
void uart_dma_init(void);
#endif
