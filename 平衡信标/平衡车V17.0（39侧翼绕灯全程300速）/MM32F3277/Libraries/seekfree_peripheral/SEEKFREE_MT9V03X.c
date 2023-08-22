/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2018,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				SEEKFREE_MT9V03X
* @company			�ɶ���ɿƼ����޹�˾
* @author			��ɿƼ�(QQ3184284598)
* @version			�鿴doc��version�ļ� �汾˵��
* @Software			IAR 8.3 or MDK 5.24
* @Taobao			https://seekfree.taobao.com/
* @date				2020-11-23
* @note
* 					���߶��壺
* 					------------------------------------
* 					ģ��ܽ�			��Ƭ���ܽ�
* 					SDA(51��RX)			�鿴SEEKFREE_MT9V03X.h�ļ��е�MT9V03X_COF_UART_TX�궨��
* 					SCL(51��TX)			�鿴SEEKFREE_MT9V03X.h�ļ��е�MT9V03X_COF_UART_RX�궨��
* 					���ж�(VSY)			�鿴SEEKFREE_MT9V03X.h�ļ��е�MT9V03X_VSYNC_PIN�궨��
* 					���ж�(HREF)		�鿴SEEKFREE_MT9V03X.h�ļ��е�MT9V03X_HREF_PIN�궨��
* 					�����ж�(PCLK)		�鿴SEEKFREE_MT9V03X.h�ļ��е�MT9V03X_PCLK_PIN�궨��
* 					���ݿ�(D0-D7)		�鿴SEEKFREE_MT9V03X.h�ļ��е�MT9V03X_DATA_PIN�궨��
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

uint8 mt9v03x_finish_flag = 0;												// һ��ͼ��ɼ���ɱ�־λ
uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];

static uint8	receive[3];
static uint8	receive_num = 0;
static vuint8	uart_receive_flag;

//��Ҫ���õ�����ͷ������
int16 MT9V03X_CFG[CONFIG_FINISH][2]=
{
	{AUTO_EXP,			0},													// �Զ��ع�����		��Χ1-63 0Ϊ�ر� ����Զ��ع⿪��  EXP_TIME�������õ����ݽ����Ϊ����ع�ʱ�䣬Ҳ�����Զ��ع�ʱ�������
																			// һ������ǲ���Ҫ����������ܣ���Ϊ�������ع���һ�㶼�ȽϾ��ȣ�����������߷ǳ������ȵ�������Գ������ø�ֵ������ͼ���ȶ���
	{EXP_TIME,			450},												// �ع�ʱ��			����ͷ�յ�����Զ����������ع�ʱ�䣬������ù���������Ϊ�������������ع�ֵ
	{FPS,				50 },												// ͼ��֡��			����ͷ�յ�����Զ���������FPS���������������Ϊ������������FPS
	{SET_COL,			MT9V03X_W},											// ͼ��������		��Χ1-752     K60�ɼ���������188
	{SET_ROW,			MT9V03X_H},											// ͼ��������		��Χ1-480
	{LR_OFFSET,			-30},													// ͼ������ƫ����	��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ188 376 752ʱ�޷�����ƫ��    ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
	{UD_OFFSET,			0},													// ͼ������ƫ����	��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ120 240 480ʱ�޷�����ƫ��    ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
//	{GAIN,				32},												// ͼ������			��Χ16-64     ����������ع�ʱ��̶�������¸ı�ͼ�������̶�
	{GAIN,				32},
	{INIT,				0}													// ����ͷ��ʼ��ʼ��
};

//������ͷ�ڲ���ȡ������������
int16 GET_CFG[CONFIG_FINISH-1][2]=
{
	{AUTO_EXP,			0},													// �Զ��ع�����
	{EXP_TIME,			0},													// �ع�ʱ��
	{FPS,				0},													// ͼ��֡��
	{SET_COL,			0},													// ͼ��������
	{SET_ROW,			0},													// ͼ��������
	{LR_OFFSET,			0},													// ͼ������ƫ����
	{UD_OFFSET,			0},													// ͼ������ƫ����
	{GAIN,				0},													// ͼ������
};

//-------------------------------------------------------------------------------------------------------------------
// @brief		MT9V03X����ͷ�����жϺ���
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

	if(1==receive_num && 0XA5!=receive[0])  receive_num = 0;//�����0��λ���յ�����A5������к������裬������0��λ����
	if(3 == receive_num)//���������3�����ݣ����receive_num��uart_receive_flag��1����ʾ����˽��� 
	{
		receive_num = 0;
		uart_receive_flag = 1;
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��������ͷ�ڲ�������Ϣ
// @param		uartn			ѡ��ʹ�õĴ���
// @param		buff			����������Ϣ�ĵ�ַ
// @return		void
// @since		v1.0
// Sample usage:				���øú���ǰ���ȳ�ʼ������
//-------------------------------------------------------------------------------------------------------------------
void set_config(int16 buff[CONFIG_FINISH-1][2])
{
	uint16 temp, i;
	uint8  send_buffer[4];

	uart_receive_flag = 0;

	//���ò���  ������ο���������ֲ�
	//��ʼ��������ͷ�����³�ʼ��
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
	//�ȴ�����ͷ��ʼ���ɹ�
	while(!uart_receive_flag);
	uart_receive_flag = 0;
	while((0xff != receive[1]) || (0xff != receive[2]));
	//���ϲ��ֶ�����ͷ���õ�����ȫ�����ᱣ��������ͷ��51��Ƭ����eeprom��
	//����set_exposure_time�����������õ��ع����ݲ��洢��eeprom��
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ȡ����ͷ�ڲ�������Ϣ
// @param		uartn			ѡ��ʹ�õĴ���
// @param		buff			����������Ϣ�ĵ�ַ
// @return		void
// @since		v1.0
// Sample usage:				���øú���ǰ���ȳ�ʼ������
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

		//�ȴ����ܻش�����
		while(!uart_receive_flag);
		uart_receive_flag = 0;

		buff[i][1] = receive[1]<<8 | receive[2];
	}
}
void uart8_putbuff(u8 *buff, u32 len)
{
	while(len)																					// ѭ����������
	{
		UART8->TDR = *buff++;														// д�뷢������
		while(!(UART8->CSR & UART_CSR_TXC));										// �ȴ��������
		len--;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		��ȡ����ͷ�̼��汾
// @param		uartn			ѡ��ʹ�õĴ���
// @return		void
// @since		v1.0
// Sample usage:				���øú���ǰ���ȳ�ʼ������
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

//	//�ȴ����ܻش�����
//	while(!uart_receive_flag);
//	uart_receive_flag = 0;

//	return ((uint16)(receive[1]<<8) | receive[2]);
//}

//-------------------------------------------------------------------------------------------------------------------
// @brief		������������ͷ�ع�ʱ��
// @param		uartn			ѡ��ʹ�õĴ���
// @param		light			�����ع�ʱ��Խ��ͼ��Խ��������ͷ�յ������ݷֱ��ʼ�FPS��������ع�ʱ��������õ����ݹ�����ô����ͷ��������������ֵ
// @return		uint16			��ǰ�ع�ֵ������ȷ���Ƿ���ȷд��
// @since		v1.0
// Sample usage:				���øú���ǰ���ȳ�ʼ������
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

//	//�ȴ����ܻش�����
//	while(!uart_receive_flag);
//	uart_receive_flag = 0;

//	temp = receive[1]<<8 | receive[2];
//	return temp;
//}

//-------------------------------------------------------------------------------------------------------------------
// @brief		������ͷ�ڲ��Ĵ�������д����
// @param		uartn			ѡ��ʹ�õĴ���
// @param		addr			����ͷ�ڲ��Ĵ�����ַ
// @param		data			��Ҫд�������
// @return		uint16			�Ĵ�����ǰ���ݣ�����ȷ���Ƿ�д��ɹ�
// @since		v1.0
// Sample usage:				���øú���ǰ���ȳ�ʼ������
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

//	//�ȴ����ܻش�����
//	while(!uart_receive_flag);
//	uart_receive_flag = 0;

//	temp = receive[1]<<8 | receive[2];
//	return temp;
//}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ʼ������ͷ���ж�
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
// @brief		��ʼ������ͷ��PCLK����
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
// @brief		��ʼ������ͷ��DMA
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

//	//DMA1���߳�ʼ��
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	//DMA����ͷ��ʼ��
//	camera_dma_init(MT9V03X_DMA_CH, (uint32)MT9V03X_DATA_ADD, (uint32)camera_buffer_addr, MT9V03X_H*MT9V03X_W);
//	//�ж�����
//	nvic_init(MT9V03X_DMA_IRQN, 0x00, 0x01, ENABLE);
//}

//-------------------------------------------------------------------------------------------------------------------
// @brief		MT9V03X����ͷ��ʼ��
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				ʹ��FLEXIO�ӿڲɼ�����ͷ
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_init(void)
{
//	camera_type = CAMERA_GRAYSCALE;//������������ͷ����
	camera_buffer_addr = mt9v03x_image[0];

	// ��ʼ������ ��������ͷ
	mt9v03x_uart_init();
	
	mt9v03x_uart_rx_irq(ENABLE);
	
	systick_delay_ms(200);

	uart_receive_flag = 0;
	//�ȴ�����ͷ�ϵ��ʼ���ɹ� ��ʽ�����֣���ʱ����ͨ����ȡ���õķ�ʽ ��ѡһ
	//systick_delay_ms(1000);//��ʱ��ʽ
	get_config( GET_CFG);//��ȡ���õķ�ʽ
	
	uart_receive_flag = 0;
	set_config( MT9V03X_CFG);

	uart_receive_flag = 0;
	//��ȡ���ñ��ڲ鿴�����Ƿ���ȷ
	get_config( GET_CFG);

	//DMA��ʼ��
	mt9v03x_dma_init();
	//GPIO������ʱ����ʼ��,PLCK���ų�ʼ��
	mt9v03x_tim1_etr_init();
	//VSYNC��ʼ��
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
		UART8->IER |= UART_IER_RX;													// ʹ�ܽ�������ж�
	else
		UART8->IER &= ~(UART_IER_RX);												// �رս�������ж�
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;                                  //�жϺ�����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0x00;	//��ռ���ȼ�ֵԽС�����ȼ�Խ��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0x00;				//��ռ���ȼ�ֵԽС�����ȼ�Խ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ��
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

	//MDA���ó�ʼ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)MT9V03X_DATA_ADD;										// Դ��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)camera_buffer_addr;											// Ŀ���ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;											// ������ΪԴ
	DMA_InitStructure.DMA_BufferSize = MT9V03X_H*MT9V03X_W;													// ������ٸ�����
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;							// �����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;										// �ڴ��ַ����+1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;						// ����ÿ�δ���һ���ֽ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;								// �ڴ�ÿ�δ���һ���ֽ�
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;												// ��ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;										// ���ȼ����
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;												// ���ڴ浽�ڴ�ģʽ
	DMA_InitStructure.DMA_Auto_reload = DMA_Auto_Reload_Enable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);

	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);													// ����DMA��������ж�
	DMA_Cmd(DMA1_Channel4, ENABLE);																	// ����DMA1
	DMA_Cmd(DMA1_Channel4, DISABLE);																	// ����DMA1

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;                                  //�жϺ�����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0x00;	//��ռ���ȼ�ֵԽС�����ȼ�Խ��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0x01;				//��ռ���ȼ�ֵԽС�����ȼ�Խ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ��
	NVIC_Init(&NVIC_InitStructure);
}
void mt9v03x_tim1_etr_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOE, ENABLE);
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_1);//PE7������ͷ�������ж�
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
	TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);					//TIM��ģʽ�������źŵ����������³�ʼ���������ʹ����Ĵ����ĸ����¼�
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);		//������ʱ���ı�������

	TIM_Cmd(TIM1, ENABLE);
	TIM_DMACmd(TIM1, TIM_DMA_Trigger, ENABLE);							//ʹ��TIM_DMA

}
void mt9v03x_exti_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOE, ENABLE);
	
//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_1);
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStruct);//PE8������ͷ�ĳ��ж�
	
	NVIC_InitTypeDef NVIC_InitStructure;														// �ж����ýṹ��
	EXTI_InitTypeDef EXTI_InitStructure;														// EXTI ���ýṹ��

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_EXTI, ENABLE);
//	RCC->APB2ENR |= RCC_APB2ENR_EXTI;
	
	SYSCFG_EXTILineConfig(((0x48&0xf0) >> 4), (0x48&0x0f));										// �����ö�Ӧ�� GPIO ���� EXTI ����ʹ��
	EXTI_StructInit(&EXTI_InitStructure);														// ��ȡĬ�ϵ� EXTI ����
	EXTI_InitStructure.EXTI_Line = 0x00000001 << (0x48&0x0f);									// ���ö�Ӧ�� LINE
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;											// �����ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;													// ���ô�����ʽ
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;													// ʹ��
	EXTI_Init(&EXTI_InitStructure);																// ��ʼ�� EXTI
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;					// ���������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;								// ���������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;												// ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);																// ��ʼ���ж�����

}
//-------------------------------------------------------------------------------------------------------------------
// @brief		MT9V03X����ͷ���ж�
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				��isr.c�����ȴ�����Ӧ���жϺ�����Ȼ����øú���(֮�����������жϱ�־λ)
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_vsync(void)
{
	//ȡ����ֱͬ��
	MT9V03X_DMA_CH->CNDTR = MT9V03X_H*MT9V03X_W;// ���õ�ǰDMA�����ʣ������ ���µݼ� �üĴ���ֻ����DMA������ʱ���ġ�
	MT9V03X_DMA_CH->CCR |= DMA_CCR_EN;

	//������ֱͬ��
//	if(!mt9v03x_finish_flag){
//	MT9V03X_DMA_CH->CNDTR = MT9V03X_H*MT9V03X_W;// ���õ�ǰDMA�����ʣ������ ���µݼ� �üĴ���ֻ����DMA������ʱ���ġ�
//	MT9V03X_DMA_CH->CCR |= DMA_CCR_EN;
//	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		MT9V03X����ͷDMA����ж�
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void mt9v03x_dma(void)
{
	MT9V03X_DMA_CH->CCR &= (uint16)(~DMA_CCR_EN);							// �ر�DMA1
	mt9v03x_finish_flag = 1;												// һ��ͼ��Ӳɼ���ʼ���ɼ�������ʱ3.8MS����(50FPS��188*120�ֱ���)
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		���������ͷͼ��������λ���鿴ͼ��
// @param		uartn			ʹ�õĴ��ں�
// @param		image			��Ҫ���͵�ͼ���ַ
// @param		width			ͼ�����
// @param		height			ͼ�����
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void sendimg_03x(UART_TypeDef* uart, uint8 *image, uint16 width, uint16 height)
{
	// ��������
	uart1_putchar(uart,0x00);
	uart1_putchar(uart,0xff);
	uart1_putchar(uart,0x01);
	uart1_putchar(uart,0x01);
	// ����ͼ��
	uart1_putbuff(uart, image, width*height);
}
void uart1_putchar(UART_TypeDef* uart, uint8 dat)
{
	uart->TDR = dat;																// д�뷢������
	while(!(uart->CSR & UART_CSR_TXC));											// �ȴ��������
}
void uart1_putbuff(UART_TypeDef* uart, uint8 *buff, uint32 len)
{
	while(len)																					// ѭ����������
	{
		uart->TDR = *buff++;														// д�뷢������
		while(!(uart->CSR & UART_CSR_TXC));										// �ȴ��������
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