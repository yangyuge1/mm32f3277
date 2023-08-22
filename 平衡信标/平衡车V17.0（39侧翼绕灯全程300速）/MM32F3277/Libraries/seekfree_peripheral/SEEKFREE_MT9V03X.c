/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2018,逐飞科技
* All rights reserved.
* 技术讨论QQ群：一群：179029047(已满)  二群：244861897
*
* 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
*
* @file				SEEKFREE_MT9V03X
* @company			成都逐飞科技有限公司
* @author			逐飞科技(QQ3184284598)
* @version			查看doc内version文件 版本说明
* @Software			IAR 8.3 or MDK 5.24
* @Taobao			https://seekfree.taobao.com/
* @date				2020-11-23
* @note
* 					接线定义：
* 					------------------------------------
* 					模块管脚			单片机管脚
* 					SDA(51的RX)			查看SEEKFREE_MT9V03X.h文件中的MT9V03X_COF_UART_TX宏定义
* 					SCL(51的TX)			查看SEEKFREE_MT9V03X.h文件中的MT9V03X_COF_UART_RX宏定义
* 					场中断(VSY)			查看SEEKFREE_MT9V03X.h文件中的MT9V03X_VSYNC_PIN宏定义
* 					行中断(HREF)		查看SEEKFREE_MT9V03X.h文件中的MT9V03X_HREF_PIN宏定义
* 					像素中断(PCLK)		查看SEEKFREE_MT9V03X.h文件中的MT9V03X_PCLK_PIN宏定义
* 					数据口(D0-D7)		查看SEEKFREE_MT9V03X.h文件中的MT9V03X_DATA_PIN宏定义
* 					------------------------------------
********************************************************************************************************************/

#include "zf_systick.h"
//#include "zf_uart.h"
//#include "zf_gpio.h"
//#include "zf_exti.h"
//#include "zf_camera.h"
#include "hal_rcc.h"
#include "hal_gpio.h"
#include "hal_uart.h"
#include "hal_dma.h"
#include "hal_exti.h"
#include "hal_conf.h"
#include "SEEKFREE_MT9V03X.h"

uint8 mt9v03x_finish_flag = 0;												// 一场图像采集完成标志位
uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];

static uint8	receive[3];
static uint8	receive_num = 0;
static vuint8	uart_receive_flag;

//需要配置到摄像头的数据
int16 MT9V03X_CFG[CONFIG_FINISH][2]=
{
	{AUTO_EXP,			0},													// 自动曝光设置		范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
																			// 一般情况是不需要开启这个功能，因为比赛场地光线一般都比较均匀，如果遇到光线非常不均匀的情况可以尝试设置该值，增加图像稳定性
	{EXP_TIME,			450},												// 曝光时间			摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
	{FPS,				50 },												// 图像帧率			摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
	{SET_COL,			MT9V03X_W},											// 图像列数量		范围1-752     K60采集不允许超过188
	{SET_ROW,			MT9V03X_H},											// 图像行数量		范围1-480
	{LR_OFFSET,			-30},													// 图像左右偏移量	正值 右偏移   负值 左偏移  列为188 376 752时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
	{UD_OFFSET,			0},													// 图像上下偏移量	正值 上偏移   负值 下偏移  行为120 240 480时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
//	{GAIN,				32},												// 图像增益			范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度
	{GAIN,				32},
	{INIT,				0}													// 摄像头开始初始化
};

//从摄像头内部获取到的配置数据
int16 GET_CFG[CONFIG_FINISH-1][2]=
{
	{AUTO_EXP,			0},													// 自动曝光设置
	{EXP_TIME,			0},													// 曝光时间
	{FPS,				0},													// 图像帧率
	{SET_COL,			0},													// 图像列数量
	{SET_ROW,			0},													// 图像行数量
	{LR_OFFSET,			0},													// 图像左右偏移量
	{UD_OFFSET,			0},													// 图像上下偏移量
	{GAIN,				0},													// 图像增益
};

//-------------------------------------------------------------------------------------------------------------------
// @brief		MT9V03X摄像头串口中断函数
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
// @note
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_uart_callback(void)
{
	receive[receive_num] = UART8->RDR & 0x01FF;;
	receive_num++;

	if(1==receive_num && 0XA5!=receive[0])  receive_num = 0;//如果在0号位接收到的是A5，则进行后续步骤，继续用0号位接收
	if(3 == receive_num)//如果积攒了3个数据，清楚receive_num，uart_receive_flag置1，表示完成了接收 
	{
		receive_num = 0;
		uart_receive_flag = 1;
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		配置摄像头内部配置信息
// @param		uartn			选择使用的串口
// @param		buff			发送配置信息的地址
// @return		void
// @since		v1.0
// Sample usage:				调用该函数前请先初始化串口
//-------------------------------------------------------------------------------------------------------------------
void set_config(int16 buff[CONFIG_FINISH-1][2])
{
	uint16 temp, i;
	uint8  send_buffer[4];

	uart_receive_flag = 0;

	//设置参数  具体请参看问题锦集手册
	//开始配置摄像头并重新初始化
	for(i=0; i<CONFIG_FINISH; i++)
	{
		send_buffer[0] = 0xA5;
		send_buffer[1] = buff[i][0];
		temp = buff[i][1];
		send_buffer[2] = temp>>8;
		send_buffer[3] = (uint8)temp;

		uart8_putbuff(send_buffer,4);
		systick_delay_ms(2);
	}
	//等待摄像头初始化成功
	while(!uart_receive_flag);
	uart_receive_flag = 0;
	while((0xff != receive[1]) || (0xff != receive[2]));
	//以上部分对摄像头配置的数据全部都会保存在摄像头上51单片机的eeprom中
	//利用set_exposure_time函数单独配置的曝光数据不存储在eeprom中
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		获取摄像头内部配置信息
// @param		uartn			选择使用的串口
// @param		buff			接收配置信息的地址
// @return		void
// @since		v1.0
// Sample usage:				调用该函数前请先初始化串口
//-------------------------------------------------------------------------------------------------------------------
void get_config(int16 buff[CONFIG_FINISH-1][2])
{
	uint16 temp, i;
	uint8  send_buffer[4];

	for(i=0; i<CONFIG_FINISH-1; i++)
	{
		send_buffer[0] = 0xA5;
		send_buffer[1] = GET_STATUS;
		temp = buff[i][0];
		send_buffer[2] = temp>>8;
		send_buffer[3] = (uint8)temp;

		uart8_putbuff(send_buffer,4);

		//等待接受回传数据
		while(!uart_receive_flag);
		uart_receive_flag = 0;

		buff[i][1] = receive[1]<<8 | receive[2];
	}
}
void uart8_putbuff(u8 *buff, u32 len)
{
	while(len)																					// 循环到发送完
	{
		UART8->TDR = *buff++;														// 写入发送数据
		while(!(UART8->CSR & UART_CSR_TXC));										// 等待发送完成
		len--;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		获取摄像头固件版本
// @param		uartn			选择使用的串口
// @return		void
// @since		v1.0
// Sample usage:				调用该函数前请先初始化串口
//-------------------------------------------------------------------------------------------------------------------
//uint16 get_version(void)
//{ 
//	uint16 temp;
//	uint8  send_buffer[4];
//	send_buffer[0] = 0xA5;
//	send_buffer[1] = GET_STATUS;
//	temp = GET_VERSION;
//	send_buffer[2] = temp>>8;
//	send_buffer[3] = (uint8)temp;

//	uart8_putbuff(send_buffer,4);

//	//等待接受回传数据
//	while(!uart_receive_flag);
//	uart_receive_flag = 0;

//	return ((uint16)(receive[1]<<8) | receive[2]);
//}

//-------------------------------------------------------------------------------------------------------------------
// @brief		单独设置摄像头曝光时间
// @param		uartn			选择使用的串口
// @param		light			设置曝光时间越大图像越亮，摄像头收到后会根据分辨率及FPS计算最大曝光时间如果设置的数据过大，那么摄像头将会设置这个最大值
// @return		uint16			当前曝光值，用于确认是否正确写入
// @since		v1.0
// Sample usage:				调用该函数前请先初始化串口
//-------------------------------------------------------------------------------------------------------------------
//uint16 set_exposure_time(uint16 light)
//{
//	uint16 temp;
//	uint8  send_buffer[4];

//	send_buffer[0] = 0xA5;
//	send_buffer[1] = SET_EXP_TIME;
//	temp = light;
//	send_buffer[2] = temp>>8;
//	send_buffer[3] = (uint8)temp;

//	uart8_putbuff(send_buffer,4);

//	//等待接受回传数据
//	while(!uart_receive_flag);
//	uart_receive_flag = 0;

//	temp = receive[1]<<8 | receive[2];
//	return temp;
//}

//-------------------------------------------------------------------------------------------------------------------
// @brief		对摄像头内部寄存器进行写操作
// @param		uartn			选择使用的串口
// @param		addr			摄像头内部寄存器地址
// @param		data			需要写入的数据
// @return		uint16			寄存器当前数据，用于确认是否写入成功
// @since		v1.0
// Sample usage:				调用该函数前请先初始化串口
//-------------------------------------------------------------------------------------------------------------------
//uint16 set_mt9v03x_reg(UARTN_enum uartn, uint8 addr, uint16 data)
//{
//	uint16 temp;
//	uint8  send_buffer[4];

//	send_buffer[0] = 0xA5;
//	send_buffer[1] = SET_ADDR;
//	temp = addr;
//	send_buffer[2] = temp>>8;
//	send_buffer[3] = (uint8)temp;

//	uart8_putbuff(send_buffer,4);
//	systick_delay_ms(10);

//	send_buffer[0] = 0xA5;
//	send_buffer[1] = SET_DATA;
//	temp = data;
//	send_buffer[2] = temp>>8;
//	send_buffer[3] = (uint8)temp;

//	uart8_putbuff(send_buffer,4);

//	//等待接受回传数据
//	while(!uart_receive_flag);
//	uart_receive_flag = 0;

//	temp = receive[1]<<8 | receive[2];
//	return temp;
//}

//-------------------------------------------------------------------------------------------------------------------
// @brief		初始化摄像头场中断
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
//void mt9v03x_exti_init(void)
//{
//	gpio_init(MT9V03X_VSYNC_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
//	exti_interrupt_init(MT9V03X_VSYNC_PIN, EXTI_Trigger_Falling, 0x00, 0x00);
//	nvic_init(MT9V03X_VSYNC_IRQN, 0x00, 0x00, ENABLE);
//}

//-------------------------------------------------------------------------------------------------------------------
// @brief		初始化摄像头场PCLK输入
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
//void mt9v03x_tim1_etr_init(void)
//{
//	camera_tim_etr_init(MT9V03X_TIMETR_PCLK, CAMERA_GRAYSCALE);
//}

//-------------------------------------------------------------------------------------------------------------------
// @brief		初始化摄像头场DMA
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
//void mt9v03x_dma_init(void)
//{
//	uint8 num;
//	for(num=0; num<8; num++)
//	{
//		gpio_init((PIN_enum)(MT9V03X_DATA_PIN + num), GPI, GPIO_LOW, GPI_FLOATING_IN);
//	}

//	//DMA1总线初始化
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	//DMA摄像头初始化
//	camera_dma_init(MT9V03X_DMA_CH, (uint32)MT9V03X_DATA_ADD, (uint32)camera_buffer_addr, MT9V03X_H*MT9V03X_W);
//	//中断配置
//	nvic_init(MT9V03X_DMA_IRQN, 0x00, 0x01, ENABLE);
//}

//-------------------------------------------------------------------------------------------------------------------
// @brief		MT9V03X摄像头初始化
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				使用FLEXIO接口采集摄像头
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_init(void)
{
//	camera_type = CAMERA_GRAYSCALE;//设置连接摄像头类型
	camera_buffer_addr = mt9v03x_image[0];

	// 初始换串口 配置摄像头
	mt9v03x_uart_init();
	
	mt9v03x_uart_rx_irq(ENABLE);
	
	systick_delay_ms(200);

	uart_receive_flag = 0;
	//等待摄像头上电初始化成功 方式有两种：延时或者通过获取配置的方式 二选一
	//systick_delay_ms(1000);//延时方式
	get_config( GET_CFG);//获取配置的方式
	
	uart_receive_flag = 0;
	set_config( MT9V03X_CFG);

	uart_receive_flag = 0;
	//获取配置便于查看配置是否正确
	get_config( GET_CFG);

	//DMA初始化
	mt9v03x_dma_init();
	//GPIO触发定时器初始化,PLCK引脚初始化
	mt9v03x_tim1_etr_init();
	//VSYNC初始化
	mt9v03x_exti_init();
}

void mt9v03x_uart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	UART_InitTypeDef UART_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOE, ENABLE);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_8);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_8);
	
	//UART8_TX   GPIOE.1
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	//UART8_RX    GPIOE.0
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_UART8, ENABLE);
	
	//Baud rate
	UART_StructInit(&UART_InitStruct);
	UART_InitStruct.BaudRate = 9600;
	//The word length is in 8-bit data format.
	UART_InitStruct.WordLength = UART_WordLength_8b;
	UART_InitStruct.StopBits = UART_StopBits_1;
	//No even check bit.
	UART_InitStruct.Parity = UART_Parity_No;
	//No hardware data flow control.
	UART_InitStruct.HWFlowControl = UART_HWFlowControl_None;
	UART_InitStruct.Mode = UART_Mode_Rx | UART_Mode_Tx;

	UART_Init(UART8, &UART_InitStruct);
	//UART_ITConfig(UART1, UART_IT_TXIEN|UART_IT_RXIEN, ENABLE);
	UART_Cmd(UART8, ENABLE);
}
void mt9v03x_uart_rx_irq(u32 status)
{
	if(status)
		UART8->IER |= UART_IER_RX;													// 使能接收完成中断
	else
		UART8->IER &= ~(UART_IER_RX);												// 关闭接收完成中断
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;                                  //中断号设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0x00;	//抢占优先级值越小，优先级越高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0x00;				//抢占优先级值越小，优先级越高
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能
	NVIC_Init(&NVIC_InitStructure);
}
void mt9v03x_dma_init(void) 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOF, ENABLE);

	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_FLOATING;
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	

	RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel4);

	//MDA配置初始化
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)MT9V03X_DATA_ADD;										// 源地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)camera_buffer_addr;											// 目标地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;											// 外设作为源
	DMA_InitStructure.DMA_BufferSize = MT9V03X_H*MT9V03X_W;													// 传输多少个数据
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;							// 外设地址不增加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;										// 内存地址依次+1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;						// 外设每次传输一个字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;								// 内存每次传输一个字节
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;												// 非循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;										// 优先级最高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;												// 非内存到内存模式
	DMA_InitStructure.DMA_Auto_reload = DMA_Auto_Reload_Enable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);

	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);													// 配置DMA传输完成中断
	DMA_Cmd(DMA1_Channel4, ENABLE);																	// 开启DMA1
	DMA_Cmd(DMA1_Channel4, DISABLE);																	// 开启DMA1

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;                                  //中断号设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0x00;	//抢占优先级值越小，优先级越高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0x01;				//抢占优先级值越小，优先级越高
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能
	NVIC_Init(&NVIC_InitStructure);
}
void mt9v03x_tim1_etr_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOE, ENABLE);
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_1);//PE7是摄像头的像素中断
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM1, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	TIM_ETRConfig(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);
	MODIFY_REG(TIM1->SMCR, TIM_SMCR_TS | TIM_SMCR_SMS, ((u32)TIM_TS_ETRF) | ((u32)TIM_SlaveMode_External1));
	
	TIM_SelectInputTrigger(TIM1, TIM_TS_ETRF);
	TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);					//TIM从模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);		//启动定时器的被动触发

	TIM_Cmd(TIM1, ENABLE);
	TIM_DMACmd(TIM1, TIM_DMA_Trigger, ENABLE);							//使能TIM_DMA

}
void mt9v03x_exti_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOE, ENABLE);
	
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_1);
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStruct);//PE8是摄像头的场中断
	
	NVIC_InitTypeDef NVIC_InitStructure;														// 中断配置结构体
	EXTI_InitTypeDef EXTI_InitStructure;														// EXTI 配置结构体

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_EXTI, ENABLE);
//	RCC->APB2ENR |= RCC_APB2ENR_EXTI;
	
	SYSCFG_EXTILineConfig(((0x48&0xf0) >> 4), (0x48&0x0f));										// 先启用对应的 GPIO 组别的 EXTI 输入使能
	EXTI_StructInit(&EXTI_InitStructure);														// 获取默认的 EXTI 配置
	EXTI_InitStructure.EXTI_Line = 0x00000001 << (0x48&0x0f);									// 设置对应的 LINE
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;											// 设置中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;													// 设置触发方式
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;													// 使能
	EXTI_Init(&EXTI_InitStructure);																// 初始化 EXTI
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;					// 设置组优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;								// 设置子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;												// 使能中断
	NVIC_Init(&NVIC_InitStructure);																// 初始化中断配置

}
//-------------------------------------------------------------------------------------------------------------------
// @brief		MT9V03X摄像头场中断
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				在isr.c里面先创建对应的中断函数，然后调用该函数(之后别忘记清除中断标志位)
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_vsync(void)
{
	//取消垂直同步
	MT9V03X_DMA_CH->CNDTR = MT9V03X_H*MT9V03X_W;// 设置当前DMA传输的剩余数量 向下递减 该寄存器只能在DMA不工作时更改。
	MT9V03X_DMA_CH->CCR |= DMA_CCR_EN;

	//开启垂直同步
//	if(!mt9v03x_finish_flag){
//	MT9V03X_DMA_CH->CNDTR = MT9V03X_H*MT9V03X_W;// 设置当前DMA传输的剩余数量 向下递减 该寄存器只能在DMA不工作时更改。
//	MT9V03X_DMA_CH->CCR |= DMA_CCR_EN;
//	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		MT9V03X摄像头DMA完成中断
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_dma(void)
{
	MT9V03X_DMA_CH->CCR &= (uint16)(~DMA_CCR_EN);							// 关闭DMA1
	mt9v03x_finish_flag = 1;												// 一副图像从采集开始到采集结束耗时3.8MS左右(50FPS、188*120分辨率)
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		总钻风摄像头图像发送至上位机查看图像
// @param		uartn			使用的串口号
// @param		image			需要发送的图像地址
// @param		width			图像的列
// @param		height			图像的行
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void sendimg_03x(UART_TypeDef* uart, uint8 *image, uint16 width, uint16 height)
{
	// 发送命令
	uart1_putchar(uart,0x00);
	uart1_putchar(uart,0xff);
	uart1_putchar(uart,0x01);
	uart1_putchar(uart,0x01);
	// 发送图像
	uart1_putbuff(uart, image, width*height);
}
void uart1_putchar(UART_TypeDef* uart, uint8 dat)
{
	uart->TDR = dat;																// 写入发送数据
	while(!(uart->CSR & UART_CSR_TXC));											// 等待发送完成
}
void uart1_putbuff(UART_TypeDef* uart, uint8 *buff, uint32 len)
{
	while(len)																					// 循环到发送完
	{
		uart->TDR = *buff++;														// 写入发送数据
		while(!(uart->CSR & UART_CSR_TXC));										// 等待发送完成
		len--;
	}
}
void sendimg_03x_as_number_to_usart(uint8 *image,uint16 width, uint16 height)
{
	for(int i=0;i<width;i++)
	printf("====");
	printf("\n");
	for(int y=0;y<height;y++)
	{
		for(int x=0;x<width;x++)
		{
				printf("%3d ",*(image+width*y+x) );
		}
		printf("\n");
	}

}