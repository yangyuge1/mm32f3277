#include "encoder.h"
#include "hal_rcc.h"
#include "hal_gpio.h"
#include "reg_tim.h"
#include "hal_tim.h"
void encoder_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;
  TIM_OCInitTypeDef  TIM_OCInitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_2);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_2);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_2);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_2);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM4, ENABLE);
	//左
	TIM3->ARR = 0xFFFF;										// 装载自动重装载值
	TIM3->PSC = 0;																// 装载预分频
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_DIRECTTI;									// 输入捕获 IC1 映射到 TI1
	TIM3->CCMR1 |= TIM_CCMR1_CC2S_DIRECTTI;									// 输入捕获 IC2 映射到 TI2
	TIM3->SMCR |= TIM_SMCR_SMS_ENCODER3;										// 从模式 编码器模式 3 选择
	TIM3->CR1 |= TIM_CR1_UDIS;													// 禁止UEV事件的产生
	TIM3->CR1 |= TIM_CR1_CEN;													// 使能定时器
	//右
	TIM4->ARR = 0xFFFF;										// 装载自动重装载值
	TIM4->PSC = 0;																// 装载预分频
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_DIRECTTI;									// 输入捕获 IC1 映射到 TI1
	TIM4->CCMR1 |= TIM_CCMR1_CC2S_DIRECTTI;									// 输入捕获 IC2 映射到 TI2
	TIM4->SMCR |= TIM_SMCR_SMS_ENCODER3;										// 从模式 编码器模式 3 选择
	TIM4->CR1 |= TIM_CR1_UDIS;													// 禁止UEV事件的产生
	TIM4->CR1 |= TIM_CR1_CEN;													// 使能定时器
}

short int tim_encoder_get_count_(TIM_TypeDef* tim)
{
	short int temp = tim->CNT;
	tim->EGR |= 0x01;															// 产生更新时间重装载计数器
	return temp;
}
