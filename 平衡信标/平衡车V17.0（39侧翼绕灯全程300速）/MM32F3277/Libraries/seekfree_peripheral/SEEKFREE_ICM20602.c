/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2018,逐飞科技
* All rights reserved.
* 技术讨论QQ群：一群：179029047(已满)  二群：244861897
*
* 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
*
* @file				SEEKFREE_ICM20602
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
* 					SCL					查看SEEKFREE_IIC文件内的SEEKFREE_SCL宏定义
* 					SDA					查看SEEKFREE_IIC文件内的SEEKFREE_SDA宏定义
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
// 以下函数是使用硬件SPI通信 相比较IIC 速度比IIC快非常多
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602 SPI写寄存器
// @param		cmd				寄存器地址
// @param		val				需要写入的数据
// @return		void
// @since		v1.0
// Sample usage:
// @note		内部调用 用户无需关心
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
// @brief		ICM20602 SPI读寄存器
// @param		cmd				寄存器地址
// @param		*val			接收数据的地址
// @return		void
// @since		v1.0
// Sample usage:
// @note		内部调用 用户无需关心
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
// @brief		ICM20602 SPI多字节读寄存器
// @param		cmd				寄存器地址
// @param		*val			接收数据的地址
// @param		num				读取数量
// @return		void
// @since		v1.0
// Sample usage:
// @note		内部调用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void icm_spi_r_reg_bytes(uint8 * val, uint8 num)
{
	ICM20602_CS(0);
	spi2_mosi(val, val, num);
	ICM20602_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ICM20602自检函数
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
// @note		内部调用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static void icm20602_self3_check(void)
{
	uint8 dat = 0;

	while(0x12 != dat)																// 判断 ID 是否正确
	{
		icm_spi_r_reg_byte(ICM20602_WHO_AM_I, &dat);								// 读取ICM20602 ID
		systick_delay_ms(10);
		//卡在这里原因有以下几点
		//1 ICM20602坏了，如果是新的这样的概率极低
		//2 接线错误或者没有接好
		//3 可能你需要外接上拉电阻，上拉到3.3V
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		初始化ICM20602
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm20602_init_spi(void)
{
	uint8 val = 0x0;

	systick_delay_ms(10);  //上电延时
	
	spi2_init();	// 硬件SPI初始化
	
	gpio_init_spi2_cs();
	
//  gpio_init(ICM20602_CS_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);


	icm20602_self3_check();//检测

	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,0x80);//复位设备
	systick_delay_ms(2);
	do																				// 等待复位成功
	{
		icm_spi_r_reg_byte(ICM20602_PWR_MGMT_1,&val);
	}while(0x41 != val);

	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,		0x01);								// 时钟设置
	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,		0x00);								// 开启陀螺仪和加速度计
	icm_spi_w_reg_byte(ICM20602_CONFIG,			0x01);								// 176HZ 1KHZ
	icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,		0x07);								// 采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
	icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,	0x18);								// ±2000 dps
	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,	0x10);								// ±8g
	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2,	0x03);								// Average 4 samples   44.8HZ   //0x23 Average 16 samples
	
//	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_1,		0x01);								// 时钟设置
//	icm_spi_w_reg_byte(ICM20602_PWR_MGMT_2,		0x00);								// 开启陀螺仪和加速度计
//	icm_spi_w_reg_byte(ICM20602_CONFIG,			0x06);								// 176HZ 1KHZ
//	icm_spi_w_reg_byte(ICM20602_SMPLRT_DIV,		0x00);								// 采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
//	icm_spi_w_reg_byte(ICM20602_GYRO_CONFIG,	0x00);								// ±2000 dps
//	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG,	0x10);								// ±8g
//	icm_spi_w_reg_byte(ICM20602_ACCEL_CONFIG_2,	0x06);								// Average 4 samples   44.8HZ   //0x23 Average 16 samples
	
	
	//ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		获取ICM20602加速度计数据
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				执行该函数后，直接查看对应的变量即可
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
// @brief		获取ICM20602陀螺仪数据
// @param		NULL
// @return		void
// @since		v1.0
// Sample usage:				执行该函数后，直接查看对应的变量即可
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
// 以下函数是使用硬件SPI通信 相比较IIC 速度比IIC快非常多
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  @brief      将ICM20602数据转化为带有物理单位的数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void icm20602_data_change(void)
{
    //ICM20602_GYRO_CONFIG寄存器
    //设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131           可以转化为带物理单位的数据， 单位为：°/s
    //设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.5          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
    //设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

    //ICM20602_ACCEL_CONFIG寄存器
    //设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
    //设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
//    unit_icm_gyro_x = (float)icm_gyro_x/16.4;
//    unit_icm_gyro_y = (float)icm_gyro_y/16.4;
//    unit_icm_gyro_z = (float)icm_gyro_z/16.4;
    //将角速度原始数据换算为rad/s
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
// @brief		SPI发送接收函数
// @param		spi_n			选择SPI模块 (SPI_1-SPI_2)
// @param		modata			发送的数据缓冲区地址
// @param		midata			发送数据时接收到的数据的存储地址(不需要接收则传 NULL)
// @param		len				发送的字节数
// @param		continuous		本次通信是CS是否持续保持有效状态 1:持续保持 0:每传输完一个字节关闭CS(一般设置为1 即可)
// @return		void				
// @since		v2.0
// Sample usage:				spi_mosi(SPI_1,buf,buf,1);										//发送buff的内容，并接收到buf里，长度为1字节 通信期间CS持续拉低
//-------------------------------------------------------------------------------------------------------------------
void spi2_mosi (uint8 *modata, uint8 *midata, uint32 len)
{
//	while(!(spi_index[spi_n]->CSTAT & SPI_SR_TXEPT));											// 发送为空
//	while(len--)																				// 判断长度
//	{
//		spi_index[spi_n]->TXREG = *modata++;													// 发送数据
//		while(!(spi_index[spi_n]->CSTAT & SPI_SR_TXEPT));										// 发送为空
//		if(midata != NULL)																		// 接收有效
//		{
//			*midata++ = spi_index[spi_n]->RXREG;												// 读取数据
//		}
//	}
//以下为重构
	while(!(SPI2->CSTAT & SPI_SR_TXEPT));											// 发送为空
	while(len--)																				// 判断长度
	{
		
		SPI2 ->TXREG = *modata++;													// 发送数据
		while(!(SPI2 ->CSTAT & SPI_SR_TXEPT));										// 发送为空
		if(midata != NULL)																		// 接收有效
		{
			*midata++ = SPI2 ->RXREG;												// 读取数据
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		GPIO初始化
// @param		pin			选择的引脚 (可选择范围由 common.h 内PIN_enum枚举值确定)
// @param		mode		引脚的方向
// @param		dat			引脚初始化时设置的电平状态，输出时有效 0：低电平 1：高电平
// @param		mode		引脚的模式
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

	GPIO_ResetBits(GPIOB,GPIO_Pin_12);																	// 初始化电平设置低
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		SPI初始化
// @param		spi_n			选择SPI模块 (SPI_1-SPI_2)
// @param		cs_pin			选择SPI片选引脚
// @param		sck_pin			选择SPI时钟引脚
// @param		mosi_pin		选择SPI MOSI引脚
// @param		miso_pin		选择SPI MISO引脚
// @param		mode			SPI模式	0：CPOL=0 CPHA=0	1：CPOL=0 CPHA=1	2：CPOL=1 CPHA=0	3：CPOL=1 CPHA=1		//具体细节可自行百度
// @param		baud			设置SPI的波特率
// @return		void
// Sample usage:				spi_init(SPI_1, SPI1_SCK_A05, SPI1_MOSI_A07, SPI1_MISO_A06, SPI1_NSS_A04, 0, 1*1000*1000);	//硬件SPI初始化  模式0 波特率为1Mhz
//-------------------------------------------------------------------------------------------------------------------
void spi2_init(void)
{
	spi2_pin_init();											// 初始化相关引脚
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_SPI2, ENABLE);
	SPI2->GCTL |= SPI_GCR_MODE;														// 主机模式
	SPI2->CCTL |= SPI_CCR_SPILEN;													// 8bits 数据
	SPI2->CCTL &= ~SPI_CCR_CPOL;											// SCK 空闲时低电平
	SPI2->CCTL |= SPI_CCR_CPHA;												// 第一个时钟边沿开始采样
	SPI2->GCTL &= ~SPI_GCR_NSS;														// 片选软件控制
	SPI2->CCTL |= SPI_CCR_RXEDGE;													// 在传输数据位的尾时钟沿采样数据 用于高速模式
	SPI2->SPBRG = 4;											// 设置波特率
	SPI2->CCTL &= ~SPI_CCR_LSBFE;													// MSB
	SPI2->GCTL |= SPI_GCR_TXEN | SPI_GCR_RXEN;										// 使能 TX/RX
	SPI2->GCTL |= SPI_GCR_SPIEN;													// 使能
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		SPI 引脚初始化 内部调用
// @param		tx_pin			选择 TX 引脚
// @param		rx_pin			选择 RX 引脚
// @param		rx_pin			选择 RX 引脚
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
