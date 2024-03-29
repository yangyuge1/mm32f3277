/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2020,逐飞科技
* All rights reserved.
* 技术讨论QQ群：三群：824575535
*
* 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
*
* @file       		2.0寸IPS屏幕
* @company	   		成都逐飞科技有限公司
* @author     		逐飞科技(QQ3184284598)
* @version    		查看doc内version文件 版本说明
* @Software 		ADS v1.2.2
* @Target core		TC264D
* @Taobao   		https://seekfree.taobao.com/
* @date       		2020-3-23
* @note
********************************************************************************************************************/

#ifndef _SEEKFREE_IPS200_PARALLEL8_H
#define _SEEKFREE_IPS200_PARALLEL8_H

#include "common.h"
#include "SEEKFREE_FONT.h"

     
// 常用颜色在SEEKFREE_FONT.h文件中定义
#define IPS200_BGCOLOR		WHITE	//背景颜色
#define IPS200_PENCOLOR		RED		//画笔颜色

// 控制引脚  FSMC固定引脚 不可更改
#define IPS200_RS_PIN  	D13
#define IPS200_CS_PIN  	D7

// 控制引脚 可以修改
#define IPS200_RD_PIN  	D4
#define IPS200_WR_PIN  	D6
#define IPS200_RST_PIN 	D5
#define IPS200_BL_PIN  	D11

// 数据引脚 FSMC固定引脚 不可更改
#define IPS200_D0_PIN 		E11
#define IPS200_D1_PIN 		E12
#define IPS200_D2_PIN 		E13
#define IPS200_D3_PIN 		E14
#define IPS200_D4_PIN 		E15
#define IPS200_D5_PIN 		D8
#define IPS200_D6_PIN 		D9
#define IPS200_D7_PIN 		D10

// 数据对应地址
#define IPS200_DATA_ADD	0x60080000
#define IPS200_CMD_ADD		0x60000000

// 控制语句
#define IPS200_RD(x)		(x? (GPIO_PIN_SET(IPS200_RD_PIN)): (GPIO_PIN_RESET(IPS200_RD_PIN)))
#define IPS200_WR(x)     	(x? (GPIO_PIN_SET(IPS200_WR_PIN)): (GPIO_PIN_RESET(IPS200_WR_PIN)))
#define IPS200_RST(x)     	(x? (GPIO_PIN_SET(IPS200_RST_PIN)): (GPIO_PIN_RESET(IPS200_RST_PIN)))
#define IPS200_BL(x)		(x? (GPIO_PIN_SET(IPS200_BL_PIN)): (GPIO_PIN_RESET(IPS200_BL_PIN)))


// 屏幕分辨率 不可修改
#define IPS200_W			240
#define IPS200_H			320


//定义显示方向
//0 竖屏模式
//1 竖屏模式  旋转180°
//2 横屏模式
//3 横屏模式  旋转180°
#define IPS200_DISPLAY_DIR 0

#if (0==IPS200_DISPLAY_DIR || 1==IPS200_DISPLAY_DIR)

#define	IPS200_X_MAX	IPS200_W	//液晶X方宽度
#define IPS200_Y_MAX	IPS200_H	//液晶Y方宽度

#elif (2==IPS200_DISPLAY_DIR || 3==IPS200_DISPLAY_DIR)

#define	IPS200_X_MAX	IPS200_H	//液晶X方宽度
#define IPS200_Y_MAX	IPS200_W	//液晶Y方宽度

#else

#error "IPS200_DISPLAY_DIR 定义错误"

#endif

void ips200_init(void); //初始化硬件
void ips200_w_data(uint8 dat);
void ips200_wr_reg(uint8 com);
void ips200_wr_data(uint8 dat);
void ips200_wr_data16(uint16 dat);
void ips200_w_reg(uint8 com,uint8 dat);
void ips200_address_set(uint16 x1,uint16 y1,uint16 x2,uint16 y2);
void ips200_clear(uint16 color);
void ips200_drawpoint(uint16 x,uint16 y,uint16 color);
void ips200_showchar(uint16 x,uint16 y,const int8 dat);
void ips200_showstr(uint16 x,uint16 y,const int8 dat[]);


void ips200_showint8(uint16 x,uint16 y,int8 dat);
void ips200_showuint8(uint16 x,uint16 y,uint8 dat);
void ips200_showint16(uint16 x,uint16 y,int16 dat);
void ips200_showuint16(uint16 x,uint16 y,uint16 dat);
void ips200_showint32(uint16 x,uint16 y,int dat,uint8 num);
void ips200_showfloat(uint16 x,uint16 y,double dat,int8 num,int8 pointnum);
void ips200_showimage(uint16 x,uint16 y,uint16 w,uint16 l,const unsigned char *p);

void ips200_displayimage032(uint8 *p, uint16 width, uint16 height);
void ips200_displayimage032_zoom(uint8 *p, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);
void ips200_displayimage032_zoom1(const uint8 *p, uint16 width, uint16 height, uint16 start_x, uint16 start_y, uint16 dis_width, uint16 dis_height);
void ips200_displayimage8660_zoom(uint16 *p, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);
void ips200_displayimage8660_zoom1(uint16 *p, uint16 width, uint16 height, uint16 start_x, uint16 start_y, uint16 dis_width, uint16 dis_height);
void ips200_displayimage8660_grayscale_zoom(uint16 *p, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);
void ips200_displayimage7725(uint8 *p, uint16 width, uint16 height);
void ips200_display_chinese(uint16 x, uint16 y, uint8 size, const uint8 *p, uint8 number, uint16 color);
//自定义函数，将显示十字架
void ips200_displayimage032_zoom1_(const uint8 *p, uint16 width, uint16 height, uint16 start_x, uint16 start_y, uint16 dis_width, uint16 dis_height, uint16 x, uint16 y,uint16 bounder,uint16 aim);
//灰度分析
void analyse(const uint8 *p,int width,int height,int x_top,int x_bottom,int y_top,int y_bottom,uint8 *out ,int x_max,int y_max,int start_x,int start_y);
void dissection_y(const uint8 *p,uint16 width,uint16 height,uint16 y, uint16 dis_width, uint16 dis_height,uint16 start_x,uint16 start_y);
void dissection_x(const uint8 *p,uint16 width,uint16 height,uint16 x, uint16 dis_width, uint16 dis_height,uint16 start_x,uint16 start_y);

#endif

