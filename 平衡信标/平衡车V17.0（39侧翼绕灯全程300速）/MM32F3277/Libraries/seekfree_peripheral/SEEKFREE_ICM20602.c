/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2018,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				SEEKFREE_ICM20602
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
* 					SCL					�鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SCL�궨��
* 					SDA					�鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SDA�궨��
* 					------------------------------------
********************************************************************************************************************/

#include "zf_systick.h"
//#include "zf_gpio.h"
#include "hal_gpio.h"
#include "hal_rcc.h"
//#include "zf_spi.h"
//#include "SEEKFREE_IIC.h"
#include "SEEKFREE_ICM20602.h"

int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
int16 icm_acc_x,icm_acc_y,icm_acc_z;
float unit_icm_gyro_x;
float unit_icm_gyro_y;
float unit_icm_gyro_z;

float unit_icm_acc_x;
float unit_icm_acc_y;
float unit_icm_acc_z;

#define Gyro_Gr 0.0010642251355f
//#define Gyro_Gr 0.00013323124f
//-------------------------------------------------------------------------------------------------------------------
// ���º�����ʹ��Ӳ��SPIͨ�� ��Ƚ�IIC �ٶȱ�IIC��ǳ���
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602 SPIд�Ĵ���
// @param		cmd				�Ĵ�����ַ
// @param		val				��Ҫд�������
// @return		void
// @since		v1.0
// Sample usage:
// @note		�ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void icm_spi_w_reg_byte(uint8 cmd, uint8 val)
{
	uint8 dat[2];
	ICM20602_CS(0);
	dat[0] = cmd | ICM20602_SPI_W;
	dat[1] = val;

	spi2_mosi(dat, dat, 2);
	ICM20602_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602 SPI���Ĵ���
// @param		cmd				�Ĵ�����ַ
// @param		*val			�������ݵĵ�ַ
// @return		void
// @since		v1.0
// Sample usage:
// @note		�ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void icm_spi_r_reg_byte(uint8 cmd, uint8 *val)
{
	uint8 dat[2];
	ICM20602_CS(0);
	dat[0] = cmd | ICM20602_SPI_R;
	dat[1] = *val;
	
	spi2_mosi(dat, dat, 2);

	*val = dat[1];
	ICM20602_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602 SPI���ֽڶ��Ĵ���
// @param		cmd				�Ĵ�����ַ
// @param		*val			�������ݵĵ�ַ
// @param		num				��ȡ����
// @return		void
// @since		v1.0
// Sample usage:
// @note		�ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void icm_spi_r_reg_bytes(uint8 * val, uint8 num)
{
	ICM20602_CS(0);
	spi2_mosi(val, val, num);
	ICM20602_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602�Լ캯��
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
// @note		�ڲ����� �û��������
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_self3_check(void)
{
	uint8 dat = 0;

	while(0x12 != dat)																// �ж� ID �Ƿ���ȷ
	{
		icm_spi_r_reg_byte(ICM20602_WHO_AM_I, &dat);								// ��ȡICM20602 ID
		systick_delay_ms(10);
		//��������ԭ�������¼���
		//1 ICM20602���ˣ�������µ������ĸ��ʼ���
		//2 ���ߴ������û�нӺ�
		//3 ��������Ҫ����������裬������3.3V
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ʼ��ICM20602
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_spi(void)
{
	uint8 val = 0x0;

	systick_delay_ms(10);  //�ϵ���ʱ
	
	spi2_init();	// Ӳ��SPI��ʼ��
	
	gpio_init_spi2_cs();
	
//  gpio_init(ICM20602_CS_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);


	icm20602_self3_check();//���

	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80);//��λ�豸
	systick_delay_ms(2);
	do																				// �ȴ���λ�ɹ�
	{
		icm_spi_r_reg_byte(ICM20602_PWR_MGMT_1,&val);
	}while(0x41 != val);

	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,		0x01);								// ʱ������
	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,		0x00);								// ���������Ǻͼ��ٶȼ�
	icm_spi_w_reg_byte(ICM20602_CONFIG,			0x01);								// 176HZ 1KHZ
	icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,		0x07);								// �������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
	icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,	0x18);								// ��2000 dps
	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,	0x10);								// ��8g
	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2,	0x03);								// Average 4 samples   44.8HZ   //0x23 Average 16 samples
	
//	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,		0x01);								// ʱ������
//	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,		0x00);								// ���������Ǻͼ��ٶȼ�
//	icm_spi_w_reg_byte(ICM20602_CONFIG,			0x06);								// 176HZ 1KHZ
//	icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,		0x00);								// �������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
//	icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,	0x00);								// ��2000 dps
//	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,	0x10);								// ��8g
//	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2,	0x06);								// Average 4 samples   44.8HZ   //0x23 Average 16 samples
	
	
	//ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ȡICM20602���ٶȼ�����
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_accdata_spi(void)
{
	struct
	{
		uint8 reg;
		uint8 dat[6];
	}buf;

	buf.reg = ICM20602_ACCEL_XOUT_H | ICM20602_SPI_R;

	icm_spi_r_reg_bytes(&buf.reg, 7);
	icm_acc_x = (int16)(((uint16)buf.dat[0]<<8 | buf.dat[1]));
	icm_acc_y = (int16)(((uint16)buf.dat[2]<<8 | buf.dat[3]));
	icm_acc_z = (int16)(((uint16)buf.dat[4]<<8 | buf.dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ȡICM20602����������
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm20602_gyro_spi(void)
{
	struct
	{
		uint8 reg;
		uint8 dat[6];
	}buf;

	buf.reg = ICM20602_GYRO_XOUT_H | ICM20602_SPI_R;

	icm_spi_r_reg_bytes(&buf.reg, 7);
	icm_gyro_x = (int16)(((uint16)buf.dat[0]<<8 | buf.dat[1]));
	icm_gyro_y = (int16)(((uint16)buf.dat[2]<<8 | buf.dat[3]));
	icm_gyro_z = (int16)(((uint16)buf.dat[4]<<8 | buf.dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// ���º�����ʹ��Ӳ��SPIͨ�� ��Ƚ�IIC �ٶȱ�IIC��ǳ���
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ICM20602����ת��Ϊ��������λ������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void icm20602_data_change(void)
{
    //ICM20602_GYRO_CONFIG�Ĵ���
    //����Ϊ:0x00 ����������Ϊ:��250 dps     ��ȡ�������������ݳ���131           ����ת��Ϊ������λ�����ݣ� ��λΪ����/s
    //����Ϊ:0x08 ����������Ϊ:��500 dps     ��ȡ�������������ݳ���65.5          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x10 ����������Ϊ:��1000dps     ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
    //����Ϊ:0x18 ����������Ϊ:��2000dps     ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s

    //ICM20602_ACCEL_CONFIG�Ĵ���
    //����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x08 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x10 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
    //����Ϊ:0x18 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
//    unit_icm_gyro_x = (float)icm_gyro_x/16.4;
//    unit_icm_gyro_y = (float)icm_gyro_y/16.4;
//    unit_icm_gyro_z = (float)icm_gyro_z/16.4;
    //�����ٶ�ԭʼ���ݻ���Ϊrad/s
    unit_icm_gyro_x = (float)icm_gyro_x*Gyro_Gr;
    unit_icm_gyro_y = (float)icm_gyro_y*Gyro_Gr;
    unit_icm_gyro_z = (float)icm_gyro_z*Gyro_Gr;
	
    unit_icm_acc_x = ((float)icm_acc_x/4096)*0.1885 +unit_icm_acc_x*0.8115;
    unit_icm_acc_y = ((float)icm_acc_y/4096)*0.1885 +unit_icm_acc_y*0.8115;
    unit_icm_acc_z = ((float)icm_acc_z/4096)*0.1885 +unit_icm_acc_z*0.8115;
	
//    unit_icm_acc_x = ((float)icm_acc_x/4096)*0.1885 +unit_icm_acc_x*0.8115;
//    unit_icm_acc_y = ((float)icm_acc_y/4096)*0.1885 +unit_icm_acc_y*0.8115;
//    unit_icm_acc_z = ((float)icm_acc_z/4096)*0.1885 +unit_icm_acc_z*0.8115;
//    unit_icm_acc_x = ((float)icm_acc_x/4096);
//    unit_icm_acc_y = ((float)icm_acc_y/4096);
//    unit_icm_acc_z = ((float)icm_acc_z/4096);

}

//-------------------------------------------------------------------------------------------------------------------
// @brief		SPI���ͽ��պ���
// @param		spi_n			ѡ��SPIģ�� (SPI_1-SPI_2)
// @param		modata			���͵����ݻ�������ַ
// @param		midata			��������ʱ���յ������ݵĴ洢��ַ(����Ҫ������ NULL)
// @param		len				���͵��ֽ���
// @param		continuous		����ͨ����CS�Ƿ����������Ч״̬ 1:�������� 0:ÿ������һ���ֽڹر�CS(һ������Ϊ1 ����)
// @return		void				
// @since		v2.0
// Sample usage:				spi_mosi(SPI_1,buf,buf,1);										//����buff�����ݣ������յ�buf�����Ϊ1�ֽ� ͨ���ڼ�CS��������
//-------------------------------------------------------------------------------------------------------------------
void spi2_mosi (uint8 *modata, uint8 *midata, uint32 len)
{
//	while(!(spi_index[spi_n]->CSTAT & SPI_SR_TXEPT));											// ����Ϊ��
//	while(len--)																				// �жϳ���
//	{
//		spi_index[spi_n]->TXREG = *modata++;													// ��������
//		while(!(spi_index[spi_n]->CSTAT & SPI_SR_TXEPT));										// ����Ϊ��
//		if(midata != NULL)																		// ������Ч
//		{
//			*midata++ = spi_index[spi_n]->RXREG;												// ��ȡ����
//		}
//	}
//����Ϊ�ع�
	while(!(SPI2->CSTAT & SPI_SR_TXEPT));											// ����Ϊ��
	while(len--)																				// �жϳ���
	{
		
		SPI2 ->TXREG = *modata++;													// ��������
		while(!(SPI2 ->CSTAT & SPI_SR_TXEPT));										// ����Ϊ��
		if(midata != NULL)																		// ������Ч
		{
			*midata++ = SPI2 ->RXREG;												// ��ȡ����
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		GPIO��ʼ��
// @param		pin			ѡ������� (��ѡ��Χ�� common.h ��PIN_enumö��ֵȷ��)
// @param		mode		���ŵķ���
// @param		dat			���ų�ʼ��ʱ���õĵ�ƽ״̬�����ʱ��Ч 0���͵�ƽ 1���ߵ�ƽ
// @param		mode		���ŵ�ģʽ
// @return		void
// Sample usage:			gpio_init(D1, GPI, GPIO_HIGH, GPI_PULL_UP);
//-------------------------------------------------------------------------------------------------------------------
void gpio_init_spi2_cs(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);
	//spi2_cs  pb12
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_ResetBits(GPIOB,GPIO_Pin_12);																	// ��ʼ����ƽ���õ�
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		SPI��ʼ��
// @param		spi_n			ѡ��SPIģ�� (SPI_1-SPI_2)
// @param		cs_pin			ѡ��SPIƬѡ����
// @param		sck_pin			ѡ��SPIʱ������
// @param		mosi_pin		ѡ��SPI MOSI����
// @param		miso_pin		ѡ��SPI MISO����
// @param		mode			SPIģʽ	0��CPOL=0 CPHA=0	1��CPOL=0 CPHA=1	2��CPOL=1 CPHA=0	3��CPOL=1 CPHA=1		//����ϸ�ڿ����аٶ�
// @param		baud			����SPI�Ĳ�����
// @return		void
// Sample usage:				spi_init(SPI_1, SPI1_SCK_A05, SPI1_MOSI_A07, SPI1_MISO_A06, SPI1_NSS_A04, 0, 1*1000*1000);	//Ӳ��SPI��ʼ��  ģʽ0 ������Ϊ1Mhz
//-------------------------------------------------------------------------------------------------------------------
void spi2_init(void)
{
	spi2_pin_init();											// ��ʼ���������
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_SPI2, ENABLE);
	SPI2->GCTL |= SPI_GCR_MODE;														// ����ģʽ
	SPI2->CCTL |= SPI_CCR_SPILEN;													// 8bits ����
	SPI2->CCTL &= ~SPI_CCR_CPOL;											// SCK ����ʱ�͵�ƽ
	SPI2->CCTL |= SPI_CCR_CPHA;												// ��һ��ʱ�ӱ��ؿ�ʼ����
	SPI2->GCTL &= ~SPI_GCR_NSS;														// Ƭѡ�������
	SPI2->CCTL |= SPI_CCR_RXEDGE;													// �ڴ�������λ��βʱ���ز������� ���ڸ���ģʽ
	SPI2->SPBRG = 4;											// ���ò�����
	SPI2->CCTL &= ~SPI_CCR_LSBFE;													// MSB
	SPI2->GCTL |= SPI_GCR_TXEN | SPI_GCR_RXEN;										// ʹ�� TX/RX
	SPI2->GCTL |= SPI_GCR_SPIEN;													// ʹ��
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		SPI ���ų�ʼ�� �ڲ�����
// @param		tx_pin			ѡ�� TX ����
// @param		rx_pin			ѡ�� RX ����
// @param		rx_pin			ѡ�� RX ����
// @return		void			NULL
// Sample usage:				spi_pin_init(sck_pin, mosi_pin, miso_pin, cs_pin);
//-------------------------------------------------------------------------------------------------------------------
static void spi2_pin_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_5);
	//spi2_sck  pb13
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	//spi2_mosi  pb15
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	//spi2_miso  pb14
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}
