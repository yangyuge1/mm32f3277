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

#ifndef _SEEKFREE_ICM20602_h
#define _SEEKFREE_ICM20602_h

//#include "common.h"
//数据类型声明
typedef unsigned char		uint8;													//  8 bits 
typedef unsigned short int	uint16;													// 16 bits 
typedef unsigned long int	uint32;													// 32 bits 
typedef unsigned long long	uint64;													// 64 bits 

typedef char				int8;													//  8 bits 
typedef short int			int16;													// 16 bits 
typedef long  int			int32;													// 32 bits 
typedef long  long			int64;													// 64 bits 

typedef volatile int8		vint8;													//  8 bits 
typedef volatile int16		vint16;													// 16 bits 
typedef volatile int32		vint32;													// 32 bits 
typedef volatile int64		vint64;													// 64 bits 

typedef volatile uint8		vuint8;													//  8 bits 
typedef volatile uint16		vuint16;												// 16 bits 
typedef volatile uint32		vuint32;												// 32 bits 
typedef volatile uint64		vuint64;												// 64 bits 

//------------------------硬件SPI--------------------------//
#define ICM20602_SPI			SPI_2
#define ICM20602_SCK_PIN		SPI2_SCK_B13		//接模块SPC
#define ICM20602_MOSI_PIN		SPI2_MOSI_B15		//接模块SDI
#define ICM20602_MISO_PIN		SPI2_MISO_B14		//接模块SDO
#define ICM20602_CS_PIN			B12					//接模块CS

#define ICM20602_CS(x)		(x? (GPIO_SetBits(GPIOB,GPIO_Pin_12)): (GPIO_ResetBits(GPIOB,GPIO_Pin_12)))
//------------------------硬件SPI--------------------------//

// ======================= 本部分参数不允许用户修改 =======================
#define ICM20602_DEV_ADDR			0x69 //SA0接地：0x68   SA0上拉：0x69  模块默认上拉
#define ICM20602_SPI_W				0x00
#define ICM20602_SPI_R				0x80

#define ICM20602_XG_OFFS_TC_H		0x04
#define ICM20602_XG_OFFS_TC_L		0x05
#define ICM20602_YG_OFFS_TC_H		0x07
#define ICM20602_YG_OFFS_TC_L		0x08
#define ICM20602_ZG_OFFS_TC_H		0x0A
#define ICM20602_ZG_OFFS_TC_L		0x0B
#define ICM20602_SELF_TEST_X_ACCEL	0x0D
#define ICM20602_SELF_TEST_Y_ACCEL	0x0E
#define ICM20602_SELF_TEST_Z_ACCEL	0x0F
#define ICM20602_XG_OFFS_USRH		0x13
#define ICM20602_XG_OFFS_USRL		0x14
#define ICM20602_YG_OFFS_USRH		0x15
#define ICM20602_YG_OFFS_USRL		0x16
#define ICM20602_ZG_OFFS_USRH		0x17
#define ICM20602_ZG_OFFS_USRL		0x18
#define ICM20602_SMPLRT_DIV			0x19
#define ICM20602_CONFIG				0x1A
#define ICM20602_GYRO_CONFIG		0x1B
#define ICM20602_ACCEL_CONFIG		0x1C
#define ICM20602_ACCEL_CONFIG_2		0x1D
#define ICM20602_LP_MODE_CFG		0x1E
#define ICM20602_ACCEL_WOM_X_THR	0x20
#define ICM20602_ACCEL_WOM_Y_THR	0x21
#define ICM20602_ACCEL_WOM_Z_THR	0x22
#define ICM20602_FIFO_EN			0x23
#define ICM20602_FSYNC_INT			0x36
#define ICM20602_INT_PIN_CFG		0x37
#define ICM20602_INT_ENABLE			0x38
#define ICM20602_FIFO_WM_INT_STATUS	0x39 
#define ICM20602_INT_STATUS			0x3A
#define ICM20602_ACCEL_XOUT_H		0x3B
#define ICM20602_ACCEL_XOUT_L		0x3C
#define ICM20602_ACCEL_YOUT_H		0x3D
#define ICM20602_ACCEL_YOUT_L		0x3E
#define ICM20602_ACCEL_ZOUT_H		0x3F
#define ICM20602_ACCEL_ZOUT_L		0x40
#define ICM20602_TEMP_OUT_H			0x41
#define ICM20602_TEMP_OUT_L			0x42
#define ICM20602_GYRO_XOUT_H		0x43
#define ICM20602_GYRO_XOUT_L		0x44
#define ICM20602_GYRO_YOUT_H		0x45
#define ICM20602_GYRO_YOUT_L		0x46
#define ICM20602_GYRO_ZOUT_H		0x47
#define ICM20602_GYRO_ZOUT_L		0x48
#define ICM20602_SELF_TEST_X_GYRO	0x50
#define ICM20602_SELF_TEST_Y_GYRO	0x51
#define ICM20602_SELF_TEST_Z_GYRO	0x52
#define ICM20602_FIFO_WM_TH1		0x60
#define ICM20602_FIFO_WM_TH2		0x61
#define ICM20602_SIGNAL_PATH_RESET	0x68
#define ICM20602_ACCEL_INTEL_CTRL	0x69
#define ICM20602_USER_CTRL			0x6A
#define ICM20602_PWR_MGMT_1			0x6B
#define ICM20602_PWR_MGMT_2			0x6C
#define ICM20602_I2C_IF				0x70
#define ICM20602_FIFO_COUNTH		0x72
#define ICM20602_FIFO_COUNTL		0x73
#define ICM20602_FIFO_R_W			0x74
#define ICM20602_WHO_AM_I			0x75
#define ICM20602_XA_OFFSET_H		0x77
#define ICM20602_XA_OFFSET_L		0x78
#define ICM20602_YA_OFFSET_H		0x7A
#define ICM20602_YA_OFFSET_L		0x7B
#define ICM20602_ZA_OFFSET_H		0x7D
#define ICM20602_ZA_OFFSET_L		0x7E
// ======================= 本部分参数不允许用户修改 =======================

extern int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
extern int16 icm_acc_x,icm_acc_y,icm_acc_z;
void icm20602_data_change(void);

//--------硬件SPI--------------
void icm20602_init_spi			  (void);
void get_icm20602_accdata_spi	(void);
void get_icm20602_gyro_spi		(void);
void spi2_mosi(uint8 *modata, uint8 *midata, uint32 len);
void gpio_init_spi2_cs(void);
void spi2_init(void);
static void spi2_pin_init (void);

#endif
