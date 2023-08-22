#include "imu963ra.h"
#include "zf_gpio.h"
#include "hal_rcc.h"
#include "SEEKFREE_ICM20602.h"

int16 imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z;
int16 imu963ra_acc_x,  imu963ra_acc_y,  imu963ra_acc_z;
int16 imu963ra_mag_x,  imu963ra_mag_y,  imu963ra_mag_z;
//-------------------------------------------------------------------------------------------------------------------
// @brief       SPI2 接口向传感器寄存器传输 8bit 数据
// @param       register_name   地址       
// @param       data            数据    
// @return      void
// Sample usage:                spi_write_8bit_register(0x11, 0x01);
//-------------------------------------------------------------------------------------------------------------------
void spi_write_8bit_register (const uint8 register_name, const uint8 data)
{
		while(!(SPI2->CSTAT & SPI_SR_TXEPT));	//发送为空
    SPI2 ->TXREG = register_name;	        //发送地址                                  
    while(!(SPI2->CSTAT & SPI_SR_TXEPT));//发送为空                                     
    SPI2 ->TXREG = data;                  //发送数据                              
    while(!(SPI2->CSTAT & SPI_SR_TXEPT));//发送为空                                          
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       SPI2 接口读取传感器寄存器 8bit 数据
// @param       register_name   地址    
// @return      uint8           数据  
// Sample usage:                spi_read_8bit_register(0x11);
//-------------------------------------------------------------------------------------------------------------------
uint8 spi_read_8bit_register ( const uint8 register_name)
{
		uint8 data;
		while(!(SPI2->CSTAT & SPI_SR_TXEPT));//发送为空
    SPI2 ->TXREG = register_name;	        //发送地址 
		while(!(SPI2->CSTAT & SPI_SR_TXEPT));//发送为空	
    SPI2 ->TXREG = 0;                     //发送空数据                              
    while(!(SPI2->CSTAT & SPI_SR_TXEPT));//等待接收                                     
    data = SPI2 ->RXREG;                  //接收数据                             
    return data;
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       SPI2 接口读取传感器寄存器 len数据
// @param       register_name   地址 
// @param       *data           数据缓冲区
// @param       len             缓冲区长度
// @return      void
// Sample usage:                spi_read_8bit_registers(0x11, data, 32);
//-------------------------------------------------------------------------------------------------------------------
void spi_read_8bit_registers (const uint8 register_name, uint8 *data, uint32 len)
{
		while(!(SPI2->CSTAT & SPI_SR_TXEPT));
    SPI2 ->TXREG = register_name;	        //发送地址 
		while(!(SPI2->CSTAT & SPI_SR_TXEPT));//发送为空	
    while(len --)
    {
    SPI2 ->TXREG = 0;                     //发送空数据                              
    while(!(SPI2->CSTAT & SPI_SR_TXEPT));//发送为空	                                  
    *data ++ = SPI2 ->RXREG;              //接收数据     
    }
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       IMU963RA 写寄存器 内部调用
// @param       reg             寄存器地址
// @param       data            数据
// @return      void
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_write_acc_gyro_register(uint8 reg, uint8 data)
{
    IMU963RA_CS(0);
    spi_write_8bit_register(reg | IMU963RA_SPI_W, data);
    IMU963RA_CS(1);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       IMU963RA 读寄存器 内部调用
// @param       reg             寄存器地址
// @return      uint8           数据
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_read_acc_gyro_register(uint8 reg)
{
    uint8 data = 0;
    IMU963RA_CS(0);
    data = spi_read_8bit_register(reg | IMU963RA_SPI_R);
    IMU963RA_CS(1);
    return data;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       IMU963RA 读数据 内部调用
// @param       reg             寄存器地址
// @param       data            数据缓冲区
// @param       len             数据长度
// @return      void
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_read_acc_gyro_registers(uint8 reg, uint8 *data, uint32 len)
{
    IMU963RA_CS(0);
    spi_read_8bit_registers(reg | IMU963RA_SPI_R, data, len);
    IMU963RA_CS(1);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       IMU963RA作为IIC主机向磁力计写数据，内部调用
// @param       void
// @return      void
// Sample usage:                内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_write_mag_register(uint8 addr, uint8 reg, uint8 data)
{
    addr = addr << 1;
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x00);    // 从机0配置清除
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 0);   // 设置地磁计地址（注意这里需要设置8位的I2C地址） 0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);     // 需要写入的寄存器地址
    imu963ra_write_acc_gyro_register(IMU963RA_DATAWRITE_SLV0, data); // 需要写入的数据
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x4C);  // 仅在第一个周期启用通讯 开启上拉 I2C主机使能
    
    //等待通讯成功
    while(0 == (0x80 & imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER)))
    {
        systick_delay_ms(2);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       IMU963RA作为IIC主机向磁力计读数据，内部调用
// @param       void
// @return      void
// Sample usage:                内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
uint8 imu963ra_read_mag_register(uint8 addr, uint8 reg)
{
    addr = addr << 1;
    
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 1);   // 设置地磁计地址（注意这里需要设置8位的I2C地址） 0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);     // 需要读取的寄存器地址
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x01);    
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x4C);  // 仅在第一个周期启用通讯 开启上拉 I2C主机使能
    
    //等待通讯成功
    while(0 == (0x01 & imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER)))
    {
        systick_delay_ms(2);
    }
    
    return (imu963ra_read_acc_gyro_register(IMU963RA_SENSOR_HUB_1)); // 返回读取到的数据
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       IMU963RA作为IIC主机向磁力计自动写数据，内部调用
// @param       void
// @return      void
// Sample usage:                内部调用，用户无需关心
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_connect_mag(uint8 addr, uint8 reg)
{
    addr = addr << 1;
    
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 1);   // 设置地磁计地址（注意这里需要设置8位的I2C地址） 0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);     // 需要读取的寄存器地址
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x06);    
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x6C);  // 仅在第一个周期启用通讯 开启上拉 I2C主机使能
}   


//-------------------------------------------------------------------------------------------------------------------
// @brief       IMU963RA 六轴自检 内部调用
// @param       void
// @return      uint8           1-自检失败 0-自检成功
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_acc_gyro_self_check (void)
{
    uint8 dat = 0;
    uint16 timeout_count = 0;

    while(0x6B != dat)                                          // 判断 ID 是否正确
    {
        if(timeout_count++ > IMU963RA_TIMEOUT_COUNT)
            return 1;
				
        dat = imu963ra_read_acc_gyro_register(IMU963RA_WHO_AM_I);
				
        systick_delay_ms(10);
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       IMU963RA 磁力计自检 内部调用
// @param       void
// @return      uint8           1-自检失败 0-自检成功
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_mag_self_check (void)
{
    uint8 dat = 0;
    uint16 timeout_count = 0;

    while(0xff != dat)                                              // 判断 ID 是否正确
    {
        if(timeout_count++ > IMU963RA_TIMEOUT_COUNT)
            return 1;
        dat = imu963ra_read_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CHIP_ID);
        systick_delay_ms(10);
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       获取 IMU963RA 加速度计数据
// @param       void
// @return      void
// Sample usage:                执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_acc (void)
{
    uint8 dat[6];

    imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_A, dat, 6);
    imu963ra_acc_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu963ra_acc_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu963ra_acc_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// @brief       获取IMU963RA陀螺仪数据
// @param       void
// @return      void
// Sample usage:                执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_gyro (void)
{
    uint8 dat[6];

    imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_G, dat, 6);
    imu963ra_gyro_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu963ra_gyro_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu963ra_gyro_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// @brief       获取 IMU963RA 磁力计数据
// @param       void
// @return      void
// Sample usage:                执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_mag (void)
{
    uint8 temp_status;
    uint8 dat[6];

    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x40);
    temp_status = imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER);
    if(0x01 & temp_status)
    {
        imu963ra_read_acc_gyro_registers(IMU963RA_SENSOR_HUB_1, dat, 6);
        imu963ra_mag_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
        imu963ra_mag_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
        imu963ra_mag_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
    }
    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       初始化 IMU963RA
// @param       void
// @return      uint8           1-初始化失败 0-初始化成功
// Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 imu963ra_init (void)
{

    systick_delay_ms(10);                                                // 上电延时

	//初始化spi及其引脚,共用来自于icm20602的初始化函数
	spi2_init();	// 硬件SPI初始化
	gpio_init_spi2_cs();
	
	
    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);    // 关闭HUB寄存器访问
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL3_C, 0x01);            // 复位设备
    systick_delay_ms(2);                             
    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);    // 关闭HUB寄存器访问
    if(imu963ra_acc_gyro_self_check())                   
    {                   
//        printf("IMU963RA acc and gyro self check error.");                    
//        return 1;    
				while(1);
    }                   
                        
    imu963ra_write_acc_gyro_register(IMU963RA_INT1_CTRL, 0x03);         // 开启陀螺仪 加速度数据就绪中断
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x40);          // 设置加速度计量程±8G以及数据输出速率104hz 以及加速度信息从第一级滤波器输出
    //IMU963RA_CTRL1_XL寄存器
	//设置为:0x40 加速度量程为:±2G	    获取到的加速度计数据 除以16393，可以转化为带物理单位的数据，单位：g(m/s^2)
	//设置为:0x48 加速度量程为:±4G      获取到的加速度计数据 除以8197， 可以转化为带物理单位的数据，单位：g(m/s^2)
	//设置为:0x4C 加速度量程为:±8G      获取到的加速度计数据 除以4098， 可以转化为带物理单位的数据，单位：g(m/s^2)
	//设置为:0x44 加速度量程为:±16G     获取到的加速度计数据 除以2049， 可以转化为带物理单位的数据，单位：g(m/s^2)
	
	imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x6C);           // 设置陀螺仪计量程±2000dps以及数据输出速率416hz
	//ICM20602_GYRO_CONFIG寄存器
	//设置为:0x62 陀螺仪量程为:±125dps  获取到的陀螺仪数据除以228.6，   可以转化为带物理单位的数据，单位为：°/s
	//设置为:0x60 陀螺仪量程为:±250dps	获取到的陀螺仪数据除以114.3，   可以转化为带物理单位的数据，单位为：°/s
	//设置为:0x64 陀螺仪量程为:±500dps  获取到的陀螺仪数据除以57.1，    可以转化为带物理单位的数据，单位为：°/s
	//设置为:0x68 陀螺仪量程为:±1000dps 获取到的陀螺仪数据除以28.6，    可以转化为带物理单位的数据，单位为：°/s
	//设置为:0x6C 陀螺仪量程为:±2000dps 获取到的陀螺仪数据除以14.3，    可以转化为带物理单位的数据，单位为：°/s
	//设置为:0x61 陀螺仪量程为:±4000dps 获取到的陀螺仪数据除以7.1，     可以转化为带物理单位的数据，单位为：°/s
	
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL3_C, 0x44);            // 使能陀螺仪数字低通滤波器
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL4_C, 0x02);            // 使能数字低通滤波器
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL5_C, 0x00);            // 加速度计与陀螺仪四舍五入
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x00);            // 开启加速度计高性能模式 陀螺仪低通滤波 133hz
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL7_G, 0x00);            // 开启陀螺仪高性能模式 关闭高通滤波
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL9_XL, 0x01);           // 关闭I3C接口

    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x40);    // 开启HUB寄存器访问 用于配置地磁计
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x80);      // 复位I2C主机
    systick_delay_ms(2);                             
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x00);      // 清除复位标志
    systick_delay_ms(2);
    
    imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL2, 0x80);    // 复位连接的外设
    systick_delay_ms(2);
    imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL2, 0x00);
    systick_delay_ms(2);
    
    
    if(imu963ra_mag_self_check())
    {
//        printf("IMU963RA mag self check error.");
//        return 1;
				while(1);
    }

    imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL1, 0x19);    // 设置磁力计量程8G 输出速率100hz 连续模式
	//IMU963RA_MAG_ADDR寄存器
	//设置为:0x19 磁力计量程为:8G     获取到的加速度计数据 除以3000， 可以转化为带物理单位的数据，单位：G(高斯)
	//设置为:0x09 磁力计量程为:2G     获取到的加速度计数据 除以12000，可以转化为带物理单位的数据，单位：G(高斯)
	
    imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_FBR, 0x01);
    imu963ra_connect_mag(IMU963RA_MAG_ADDR, IMU963RA_MAG_OUTX_L);
    
    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);                            // 关闭HUB寄存器访问

	systick_delay_ms(20);	// 等待磁力计获取数据
	
    return 0;
}
