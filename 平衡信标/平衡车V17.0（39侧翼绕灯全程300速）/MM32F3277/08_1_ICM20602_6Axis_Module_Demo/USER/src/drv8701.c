 #include "drv8701.h"
#include "hal_rcc.h"
#include "types.h"
#include "reg_tim.h"
#include "hal_tim.h"
#include "hal_gpio.h"
#include "reg_common.h"
#include "reg_fsmc.h"
#include "mm32_device.h"
#define SystemCoreClock 120000000
////////////////////////////////////////////////////////////////////////////////
///
///
/// 
/// 
/// 
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///A0,A1方向控制
///A1,A3pwm输出
///TIM5 CH2,CH4 
///初始化函数
/// 
////////////////////////////////////////////////////////////////////////////////
void drv8701_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStruct;
  TIM_OCInitTypeDef  TIM_OCInitStruct;
	u16	psc = ((SystemCoreClock / 20000) >> 15);											// 计算预分频
	u16 arr = (SystemCoreClock / 20000 / (psc+1));								// 计算自动重装载值
	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_2);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_15);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_15);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM5, ENABLE);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
	TIM_TimeBaseStruct.TIM_Period = arr;
	TIM_TimeBaseStruct.TIM_Prescaler = psc;
	//Setting Clock Segmentation
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
	//TIM Upward Counting Mode	
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStruct);
	
	TIM_OCStructInit(&TIM_OCInitStruct);
	//Select Timer Mode: TIM Pulse Width Modulation Mode 2
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	//Setting the Pulse Value of the Capture Comparison Register to be Loaded
	TIM_OCInitStruct.TIM_Pulse = 0;
	//Output polarity: TIM output is more polar
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM5, &TIM_OCInitStruct);
	TIM_OC4Init(TIM5, &TIM_OCInitStruct);
	
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_CtrlPWMOutputs(TIM5, ENABLE);

	TIM_Cmd(TIM5, ENABLE);
	TIM_SetCompare2(TIM5, 0);
	TIM_SetCompare4(TIM5, 0);
}
////////////////////////////////////////////////////////////////////////////////
///电机控制函数
///以摄像头朝向为前方，据此区分左右
/// 
/// 
/// 
////////////////////////////////////////////////////////////////////////////////
void drv8701_control(int PWMsetLeft,int PWMsetRight)
{
	//左电机GPIO_Pin_0
  if(PWMsetRight<0)
	{
		PWMsetRight =-PWMsetRight ;
		TIM_SetCompare2(TIM5, PWMsetRight);
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
	}
	else
	{
		TIM_SetCompare2(TIM5, PWMsetRight);
    GPIO_ResetBits(GPIOA,GPIO_Pin_0);	
	}
  //右电机GPIO_Pin_2
	  if(PWMsetLeft<0)
	{
		PWMsetLeft =-PWMsetLeft ;
		TIM_SetCompare4(TIM5, PWMsetLeft);
		GPIO_ResetBits(GPIOA,GPIO_Pin_2);
	}
	else
	{
		TIM_SetCompare4(TIM5, PWMsetLeft);
    GPIO_SetBits(GPIOA,GPIO_Pin_2);	
	}
}