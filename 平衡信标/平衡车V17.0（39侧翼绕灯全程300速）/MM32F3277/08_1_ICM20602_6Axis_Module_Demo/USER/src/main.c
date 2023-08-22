/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				main
* @company			�ɶ���ɿƼ����޹�˾
* @author			��ɿƼ�(QQ3184284598)
* @version			�鿴doc��version�ļ� �汾˵��
* @Software			IAR 8.3 or MDK 5.24
* @Target core		MM32F3277
* @Taobao			https://seekfree.taobao.com/
* @date				2021-02-22
********************************************************************************************************************/
#include "math.h"
#include "string.h"
#include "hal_uart.h"
#include "hal_misc.h"
#include "hal_rcc.h"
#include "drv8701.h"
#include "encoder.h"
#include "board.h"
#include "zf_systick.h"
#include "delay.h"
#include "SEEKFREE_WIRELESS.h"
#include "SEEKFREE_MT9V03X.h"
#include "SEEKFREE_ICM20602.h"
#include "imu963ra.h"
#include "SEEKFREE_PRINTF.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "image.h"
#include "zf_gpio.h"
#include "pid.h"
#include <stdint.h>
//#include "headfile.h"
// **************************** ע����ģ�� ****************************
// ****************************  ****************************
// **************************** ע����ģ�� ****************************

// *************************** ����˵�� ***************************
// 
// ������Ҫ׼����ɿƼ� MM32F3277 ���İ�һ��
// 
// ����������Ҫ׼����ɿƼ� CMSIS-DAP ���������� �� ARM ���������� һ��
// 
// �����̳�ʼ���� �����������ǽӿ�
// 
// ��¼�����̺� ��ϵ����
// ��Ҫ��װICM20602ģ��
// ȷ���������� �ϵ� �����ʹ�õ�ع��� ��ȷ����ص�������
// �������ICM20602����
// 
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������
// 
// *************************** ����˵�� ***************************

// **************************** �������� ****************************
void gyro_OffsetInit(void);//������ԭʼ������Ʈ�ɼ�
void q0q1q2q3_init(void);
static float invSqrt(float x);
void Imu_Update(float gx,float gy,float gz,float ax,float ay,float az);
void wireless_printf(char *buff1,double data,char *buff2);
void uart_order_receive (UART_TypeDef* uart);
void tim8_interrupt_init_ms(uint32 timer, uint8 preemption_priority, uint8 sub_priority);
void tim7_interrupt_init_ms(uint32 timer, uint8 preemption_priority, uint8 sub_priority);
void show_turn(void);
int calculate_bounder(double mt9v03x_pitch,double K);
void key_action(void);
void add(int line,int location);
void reduce(int line,int location);
void blue_square(int line,int location);
double measure_distance(double K);
int calculate_aaa(float target);
void white_square(int line,int location);

float pid_calculate(float *err,float *PID,float aim,float now);
int pid_calculate_1(float *err,float *PID,float aim,int now);  
float pid_calculate_2(float *err,float *PID,float aim,int now) ;               //λ��ʽ
float pi_control_for_speed(float *err,float *PID,int aim,float now);
float pid_calculate_3(float *err,float *PID,float aim,float now);
int pid_calculate_4(float *err,float *PID,float aim,float now);
int pd_control_for_balance(float *err,float *PID,float aim,float Pitch);
int pid_calculate_3_1(float *err,float *PID,float aim,float now);              //λ��ʽ


// **************************** �������� ****************************

// **************************** �������޸Ĳ��� ****************************
//#define balance_coefficient 1.1       //ƽ�⻷Ӳ��ϵ�� �ܵ���ص�ѹӰ�� Ĭ��Ϊ1.0 Խ����Խǿ �������
//#define move_speed 120								//ֱ��Ŀ���ٶ� ����Ϊ�� ������С��30 �������ٶ���� �ٶȻ��൱�ºͣ����ٶ�Ŀ������䲻����
//#define turn_speed 130								//ת��Ŀ���ٶ� ����Ϊ�� ������С��30 
//#define turn_rate 120								  //�޸Ĵ˴��ı䷽��Ӳ�� �Ƽ�100~200 Խ��Խǿ ����̫С
//#define turn_radius 2.9f             //ת��뾶ϵ�� ����ֵԽСת��뾶ԽС ������С��1 �����Ŵ���ת�䷽�� ע�����ֺ��f ����Ϊ������
//#define balance_zero -19.5              //�޸Ĵ˴��ı�ƽ��㣬ÿ��Ӳ���ı䶼Ӧ���޸�ƽ��㣬���裺1���رյ��� 2�����ڴ�ӡʵʱPitch 3���ַ�����������ƽ�� 4�����´�ʱPitch������˴�
//#define acc_rate 7										//�޸Ĵ˴��ı�������� �Ƽ�Ϊ3 Խ��Խǿ ���ɳ���7
//#define image_aim_mid 30              //�޸Ĵ˴��ı䳯����ʻʱ����׼�� ����С��0 ���� ���ڻ�����
#define balance_coefficient 1.2        //ƽ�⻷Ӳ��ϵ�� �ܵ���ص�ѹӰ�� Ĭ��Ϊ1.0 Խ����Խǿ �������
#define speed_1 300 					//ֱ��Ŀ���ٶ� ����Ϊ�� ������С��30 �������ٶ���� �ٶȻ��൱�ºͣ����ٶ�Ŀ������䲻����
#define speed_2 50				//ת��Ŀ���ٶ� ����Ϊ�� ������С��30 
#define speed_3 300
#define speed_4 100
#define turn_rate 150								  //�޸Ĵ˴��ı䷽��Ӳ�� �Ƽ�100~200 Խ��Խǿ ����̫С
float turn_radius=-2.0f;             //ת��뾶ϵ�� ����ֵԽСת��뾶ԽС ������С��1 �����Ŵ���ת�䷽�� ע�����ֺ��f ����Ϊ������

//#define balance_zero 46.677 //����              //�޸Ĵ˴��ı�ƽ��㣬ÿ��Ӳ���ı䶼Ӧ���޸�ƽ��㣬���裺1���رյ��� 2�����ڴ�ӡʵʱPitch 3���ַ�����������ƽ�� 4�����´�ʱPitch������˴�
// #define balance_zero 27.187//�Ѿ���õĸ����Ƿ�Χ��ǰ�㼫��-1����������70��
 #define balance_zero 3.449//�Ѿ���õĸ����Ƿ�Χ��ǰ�㼫��-1����������70��

#define acc_rate 2 								//�޸Ĵ˴��ı�������� �Ƽ�Ϊ3 Խ��Խǿ ���ɳ���7
#define aim_mid 46
float image_aim_mid=aim_mid;      //�޸Ĵ˴��ı䳯����ʻʱ����׼�� ����С��0 ���� ���ڻ�����
// **************************** �������޸Ĳ��� ****************************
// **************************** �������޸Ĳ��� ****************************

// **************************** �궨�������� ****************************
// **************************** �궨�� ****************************
// **************************** �������������� ****************************
int control_fps=0,image_fps=0,move=0;
// **************************** �������� ****************************


// **************************** �궨��ͼ����� ****************************
// **************************** �궨�� ****************************
// **************************** ��������ͼ����� ****************************
uint8 i=0,j=0,l=3,b=0,c=0,lable=0,t,y,x,maxlable=1,x_average=0,x_average_last=0,y_average=0,area=0;
float density=0,scale=0;
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern uint8 mt9v03x_image_222[MT9V03X_H][w_max];
uint8 mt9v03x_image2[MT9V03X_H][MT9V03X_W];
uint8 mt9v03x_image3[MT9V03X_H][MT9V03X_W];
uint8 mt9v03x_image33[MT9V03X_H][MT9V03X_W];
uint8 mt9v03x_image4[MT9V03X_H][MT9V03X_W]={0};
uint8 zero[MT9V03X_H][MT9V03X_W]={0};  //zero�з�������ͨ���ǩһ����ֵ
int parent[1000] = {0};  															//����ʹ��ǩһ������ͨ�����
uint8 Lable[100] = {0};

uint8 map[MT9V03X_H][MT9V03X_W]={0};//�켣��¼

int bounder;
float threshhold=20;
int image_max=0;
int wide;
uint8 x_average_use=0;
//�Ҷȷ�����ר������

 int num;

float mt9v03x_angle=60;
float mt9v03x_rate=1.249;

float bounder_flag=1;

float num_max=70;
float num_min=20;

uint8 display_153x100[153][100];
uint8 display_255x100[255][100];


// **************************** �������� ****************************

// **************************** �궨��ͨ����� ****************************
#define UART1_RECEIVE_LEN  			10  	//�����������ֽ��� 200
#define UART4_RECEIVE_LEN  			16  	//�����������ֽ��� 200
// **************************** �궨�� ****************************
// **************************** �������ͨ����� ****************************
u8 UART1_RX_BUF[UART1_RECEIVE_LEN];     //���ߴ��ڽ��ջ���,���USART1_RECEIVE_LEN���ֽ�
u16 UART1_RX_STA=3;                 		 	//����״̬���
uint16 uart1_rx_index = 0;
u8 UART4_RX_BUF[UART4_RECEIVE_LEN];     //���ߴ��ڽ��ջ���,���USART1_RECEIVE_LEN���ֽ�
u16 UART4_RX_STA=0;                 		 	//����״̬���
uint16 usart4_rx_index = 0;

int orderA=1,orderB=0,orderC=1,orderD=1;				//�ⲿ���͵�����
// **************************** ������� ****************************

// **************************** �궨���������� ****************************
#define ENCODER1_TIM		TIM_3
#define ENCODER1_A			TIM_3_ENC1_B04
#define ENCODER1_B			TIM_3_ENC2_B05

#define ENCODER2_TIM		TIM_4
#define ENCODER2_A			TIM_4_ENC1_B06
#define ENCODER2_B			TIM_4_ENC2_B07
// **************************** �궨�� ****************************
// **************************** ���������������� ****************************
double encoder1_speed = 0;       //left 
double encoder2_speed = 0;       //right

double encoder1_use = 0;       //left 
double encoder2_use = 0;       //right

int16 encoder1_acc = 0;       //left 
int16 encoder2_acc = 0;       //right

// **************************** �������� ****************************

// **************************** �궨��PWM��� ****************************
#define MAX_DUTY			50														// ��� 75% ռ�ձ�
#define DIR_L				A0
#define DIR_R				A2

#define PWM_TIM				TIM_5
#define PWM_L				TIM_5_CH2_A01
#define PWM_R				TIM_5_CH4_A03
// **************************** �궨�� ****************************
// **************************** ��������PWM��� ****************************
int PWMsetLeft=0;
int PWMsetRight=0;
int PWMtest=0;
// **************************** �������� ****************************

// **************************** �궨��Ƕ���� ****************************
#define G 9.80665f					// m/s^2
//#define Kp 1.50f
//#define Ki 0.005f
double Kp=0.3;
double Ki=0.001;
#define halfT 0.0005f						//�������ڵ�һ�룬��λS
#define PI 3.1415926f						//Բ����
// **************************** �궨�� ****************************
// **************************** ��������Ƕ���� ****************************
struct GyroOffsetData
	{
		int Xdata;
		int Ydata;
	  int Zdata;
    int i;                        //��Ʈ����״̬��־
	};
struct GyroOffsetData GyroOffset={-100,-59,70,1};
extern float unit_icm_gyro_x;
extern float unit_icm_gyro_y;
extern float unit_icm_gyro_z;
extern float unit_icm_acc_x;
extern float unit_icm_acc_y;
extern float unit_icm_acc_z;
float Yaw,Pitch,Roll;                            //ŷ����
float q0 = 0, q1 =0, q2 = 0, q3 = 0;		         //��Ԫ��
float exInt = 0, eyInt = 0, ezInt = 0;           //������������ۼƻ���
float acc;																			 //ˮƽ���ٶ�
// **************************** �������� ****************************

// **************************** �궨��PID������� ****************************
// **************************** �궨�� ****************************
// **************************** ��������PID������� ****************************
float directionPID[3]={40,0,0};     //0--P;1--I;2--D
float balancePID[3]={3.3 ,0.01, 100};
//float balancePID[3]={6*balance_coefficient,0,-1*balance_coefficient};
//float balancePID[3]={0.03,0.003,-0.01};
float speedPIDMid[3]={0.128,0.004,0};
//float speedPIDMid[3]={0,0,0};
float speedPID[3]={30,0,0};
//float balancePID[3]={6*balance_coefficient,0,-1*balance_coefficient};
float speedAim=0;
float speedAimTurnSet=38;
//float speedAimMid=move_speed;
float speedAimMid=0;
	float speed_I_MAX=1000;
	float I_TIME_LENGTH=1;    //0.955

float balance_flag=-1;

float speedPIDLeft[3]={10 ,0.01, 150};
float speedPIDRight[3]={10 ,0.01, 150};
float speedAimLeft=10;
float speedAimRight=10;
int test=0;
float speed_use=0;
//float angleRatePID[3]={5,0.5,0};//   high power 16 3 
//float angleRatePID[3]={100,5,0}; //low power

//float speedPIDMid[3]={0.3,0.001,0};
//float speedPIDMid[3]={0.1,0.01,0};3ms
//float move_speedPIDMid[3]={0.23,0.02,1};//   high power
//#define a 0.55f
//float move_speedPIDMid[3]={a,a/200,0};//   high power

//float move_speedPIDMid[3]={0.1,0.1,0}; //low power

//float stop_speedPIDMid[3]={0.23,0.02,1};  //0.23--60\80    
//float stop_speedPIDMid[3]={-20,-0.3,0};  //0.23--60\80    

//float speedPID[3]={turn_rate,0,0};
//float accPID[3]={0.02,0.002,0};

float directionERR[5]={0,0,0,0,0};    //0--��ǰ���;1--�ϴ����;2--���ϴ����;3-���Ļ���;4-����΢��
float balanceERR[5]={0,0,0,0};

//float angleRateERR[5]={0,0,0,0,0};

float speedERRLeft[5]={0,0,0,0};
float speedERRRight[5]={0,0,0,0};
float speedERRMid[5]={0,0,0,0};
float speedERR[5]={0,0,0,0};
//float accERR[5]={0,0,0,0};

float YawAim=0;											//ƫ����-180~+180
float PitchAim=balance_zero;								//������
float test_Pitch;
float lastPitch=0;
float angleRateAim=0;
float angleRate=0;
float changePitchAim;
	
float RollAim=0;										//�����
float acc=0 ;
//float speedAimMid=move_speed;
float position=0;
float accAim=0 ;
float change=0,lastchange=0,change2=0,change3=0,change5=0,change4=0;
float change2_max=4,change2_min=-4;
int balance_circle=0,balance_circle_1=0,icm20602_circle=0,speed_circle=0,speed_circle_1=0,turn_flag=0,stop_move_flag=0,state=0;
int movementRight=0,movementLeft=0;
int err_add=0;
float distance=0,L=0;

float change2_MAX;
float change2_MIN;
// **************************** �������� ****************************

// **************************** �궨�尴����� ****************************
#define KEY_1				G0														// ���������ϰ�����Ӧ����
#define KEY_2				G1														// ���������ϰ�����Ӧ����
#define KEY_3				G2														// ���������ϰ�����Ӧ����
#define KEY_4				G3														// ���������ϰ�����Ӧ����
// **************************** �궨�� ****************************

// **************************** �������尴����� ****************************
uint8 key0_state = 0;																// ��������״̬
uint8 key1_state = 0;																// ��������״̬
uint8 key2_state = 0;																// ��������״̬
uint8 key3_state = 0;																// ��������״̬
uint8 key4_state = 0;																// ��������״̬
uint8 key5_state = 0;																// ��������״̬
uint8 key6_state = 0;																// ��������״̬
uint8 key7_state = 0;																// ��������״̬

uint8 key0_state_last = 0;															// ��һ�ΰ�������״̬
uint8 key1_state_last = 0;															// ��һ�ΰ�������״̬
uint8 key2_state_last = 0;															// ��һ�ΰ�������״̬
uint8 key3_state_last = 0;															// ��һ�ΰ�������״̬
uint8 key4_state_last = 0;															// ��һ�ΰ�������״̬
uint8 key5_state_last = 0;															// ��һ�ΰ�������״̬
uint8 key6_state_last = 0;															// ��һ�ΰ�������״̬
uint8 key7_state_last = 0;															// ��һ�ΰ�������״̬

uint8 key0_flag;
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;
uint8 key5_flag;
uint8 key6_flag;
uint8 key7_flag;

int line=0,location=0;
int line_max=4,line_min=0,location_max=1,location_min=-3;
#define line_max 20
float * dispaly[line_max]={&balancePID[0]
,&balancePID[2]
,&speedPIDMid[0]
,&speedPIDMid[1]
,&speedAimMid
,&threshhold
,&image_aim_mid
,&PitchAim

,&num_max
,&turn_radius
	
,&mt9v03x_angle
,&bounder_flag
,&mt9v03x_rate
	
,&directionPID[0]
,&directionPID[1]
,&directionPID[2]

,&speedPID[0]
,&speedPID[1]
,&speedPID[2]	

,&speedAimTurnSet

};

// **************************** �������� ****************************


// **************************** �������� ****************************
int main(void)
 	{
// **************************** ��ʼ�� debug ������� ****************************	
	board_init();
// **************************** ��ʼ�� ���� ������� ****************************	
	seekfree_wireless_init();
// **************************** �򿪴���1��4��������ж� ****************************	
	uart1_rx_irq();
	uart4_rx_irq();
// **************************** ��ʼ��Ӳ��SPI�ӿڵ� ICM20602 ****************************	
	icm20602_init_spi();								
// **************************** ��ʼ��Ӳ��SPI�ӿڵ� IMU963RA ****************************	
//	imu963ra_init();
// **************************** ��ʼ����Ļ ****************************
	ips200_init();
// **************************** ����ͷ��ʼ�� ****************************
  mt9v03x_init();																	// ��ʼ��ģ��
//	wireless_usart_dma();
// **************************** ��ʼ�������� ****************************
	encoder_init();	
// **************************** ��ʼ��PWM��� ****************************
	PWMsetLeft=PWMsetRight=0;
	drv8701_init();		
// **************************** ��ʼ��TIM8�ж�ģʽ���жϴ�������1ms ****************************
	tim8_interrupt_init_ms(1, 0x01, 0x01);
	tim7_interrupt_init_ms(1000,0x00,0x00);
// **************************** ��������Ʈ���� ****************************
  gyro_OffsetInit();	
// **************************** ������ʼ�� ****************************
	gpio_init(G0, GPI, GPIO_HIGH, GPI_PULL_UP);									// ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
	gpio_init(G1, GPI, GPIO_HIGH, GPI_PULL_UP);									// ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
	gpio_init(G2, GPI, GPIO_HIGH, GPI_PULL_UP);									// ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
	gpio_init(G3, GPI, GPIO_HIGH, GPI_PULL_UP);									// ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
	gpio_init(G4, GPI, GPIO_HIGH, GPI_PULL_UP);
	gpio_init(G5, GPI, GPIO_HIGH, GPI_PULL_UP);
	gpio_init(G6, GPI, GPIO_HIGH, GPI_PULL_UP);
	gpio_init(G7, GPI, GPIO_HIGH, GPI_PULL_UP);
		show_turn();
blue_square(line,location);
	while(1)
	{		
key_action();
//ת�򻷴�ӡ
//wireless_printf("y=",speedAim,",");
//wireless_printf("z=",encoder1_speed+encoder2_speed,",\n");
		
//�ٶȻ���ӡ
//wireless_printf("x=",speedAimMid,",");
//wireless_printf("x=",speed_use,",");
//wireless_printf("y=",encoder2_speed,",");
//wireless_printf("w=",encoder1_speed,",");
//wireless_printf("x=",encoder2_use,",");
//wireless_printf("z=",encoder1_use,",");
//wireless_printf("z=",(PitchAim+(change4+(change2- change4)*speed_circle/49)),",");		
		
//wireless_printf("y=",scale,",\n");
//wireless_printf("z=",0,",");
//wireless_printf("x=",change/100.0,",");
//wireless_printf("y=",change3/100.0,",\n");
		
//wireless_printf("y=",encoder1_speed+encoder2_speed,",\n");
//printf("x=%d,y=%d,z=%lf,w=%lf,\n",encoder1_speed,encoder2_speed,speedAimLeft,speedAimRight);
//printf("y=%lf,x=%lf,z=%lf\n",Pitch,Roll,unit_icm_gyro_z);
//wireless_printf("z=",0,",");
//wireless_printf("z=",x_average,",");
		
//wireless_printf("z=",unit_icm_acc_z,",");
//wireless_printf("x=",unit_icm_acc_x,",\n");
//systick_delay_ms(100);
//printf("x=%lf,y=%lf,z=%lf,r=%lf\n",Pitch,icm_gyro_y,10000000*angleRate/icm_gyro_y);
//printf("x=%lf,y=%lf,z=%lf,r=%lf\n",q0,q1,q2,q3);
//printf("x=%lf,y=%d,z=%d\n",acc,encoder1_speed,encoder1_speed);
//printf("w=%lf,\n",sqrt(unit_icm_acc_x*unit_icm_acc_x+unit_icm_acc_y*unit_icm_acc_y+unit_icm_acc_z*unit_icm_acc_z));
//printf("w=%lf,\n",encoder1_acc*0.00001);

//wireless_printf("w=",Pitch,",\n");
//wireless_printf("w=",test_Pitch,",\n");

//wireless_printf("z=",(PitchAim+(change4+(change2- change4)*speed_circle/49)),",");
//wireless_printf("w=",PWMsetRight/100.0,",");
//wireless_printf("z=",angleRate*10000,",\n");

//wireless_printf("x=",change4/100.0,",\n");
//wireless_printf("w=",PWMsetRight/100.0,",\n");

//wireless_printf("y=",speedERRMid[3],",");
//wireless_printf("x=",y_average,",\n");	
//wireless_printf("x=",x_average,",\n");
//wireless_printf("x=",encoder1_speed+encoder2_speed,",");
//wireless_printf("z=",(encoder1_speed+encoder2_speed)/10.0,","); 
//wireless_printf("y=",speedAimMid,",\n"); 
//printf("x=%lf,y=%lf,z=%lf\n",Pitch,PitchAim+change2,PWMsetLeft/100.0); 	
//printf("x=%d,y=%d,\n",x_average,y_average); 
//printf("x=%d,y=0,",MT9V03X_DMA_CH->CNDTR);
	if(mt9v03x_finish_flag)
	{
//		if(bounder_flag>=1)

//		if(orderD==2)
		bounder=calculate_bounder(mt9v03x_angle,mt9v03x_rate); 
//		else
//		bounder=0;
		
		x_average_last=x_average;
		
		num=0;
		for(i=0;i < MT9V03X_H;i++) 
		{
			for(j=0;j<MT9V03X_W;j++)
			{					
				
				zero[i][j]=0;
				mt9v03x_image4[i][j]=0;
				parent[i] = 0;
					
				if(mt9v03x_image[i][j]<threshhold)
					mt9v03x_image2[i][j]=0;
				else
					mt9v03x_image2[i][j]=255;
			}
		}
//�е�����		
//			two_pass();
//			findshape();
//			findLabel();
//			showLabel();
//		findcenter_4();	 //������
			findcenter_2();		
//		}		

//			erode();
//			dilate();

//		wide=findwide();
//		if(x_average==0)wide=0;
		if( x_average )
		{x_average_use=x_average;}
		else
		x_average_use=x_average_last;
//wireless_printf("z=",num,",\n");
//		x_average_use=x_average;
		
//		map[y_average][x_average]=255;
//		ips200_displayimage032_zoom1_(mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 2, 2, MT9V03X_W,MT9V03X_H,x_average,y_average,bounder,image_aim_mid);
//		ips200_displayimage032_zoom1(map[0],MT9V03X_W,MT9V03X_H,2,4+MT9V03X_H,MT9V03X_W,MT9V03X_H);
		
		
//		ips200_showstr(100,0,"x_:");
//	  ips200_showfloat(130+calculate_aaa(x_average_use),0,x_average_use,3,1);
//		
//		ips200_showstr(100,1,"y_:");
//	  ips200_showfloat(130+calculate_aaa(y_average),1,y_average,3,1);
//		
//		ips200_showstr(100,2,"nu:");
//	  ips200_showfloat(130+calculate_aaa(num),2,num,3,1);
//		ips200_displayimage032_zoom1_(mt9v03x_image2[0], MT9V03X_W, MT9V03X_H, 4+MT9V03X_W, 2, MT9V03X_W,MT9V03X_H,x_average_use,y_average,bounder,image_aim_mid);
//		ips200_displayimage032_zoom1_(mt9v03x_image4[0], MT9V03X_W, MT9V03X_H, 2, 4+MT9V03X_H, MT9V03X_W,MT9V03X_H,x_average_use,y_average,bounder,image_aim_mid);
//		ips200_displayimage032_zoom1_(zero[0], MT9V03X_W, MT9V03X_H, 4+MT9V03X_W, 4+MT9V03X_H, MT9V03X_W,MT9V03X_H,x_average_use,y_average,bounder,image_aim_mid);

		//���ߴ��� ���־������	
//   sendimg_03x_as_number_to_usart(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//���ߴ��� ͼ����λ����ʾ
//	 sendimg_03x(UART1, mt9v03x_image4[0], MT9V03X_W, MT9V03X_H);
//�Ҷȷֲ�ͳ�Ʒ������� �������к���
//	analyse(mt9v03x_image2[0],MT9V03X_W,MT9V03X_H,MT9V03X_W-1,0,MT9V03X_H-1,0,display_153x100[0],153,100,158,188);
//y_averageˮƽ����Ҷȷֲ��������� �������к���
//	dissection_y(mt9v03x_image[0], MT9V03X_W, MT9V03X_H,y_average,MT9V03X_W,50,2,2+MT9V03X_H+2);
//	dissection_x(mt9v03x_image[0], MT9V03X_W, MT9V03X_H,x_average,50,MT9V03X_H,2+MT9V03X_W+2,2);

		lable=0;  
		mt9v03x_finish_flag = 0;

//�����н���׼��
//	if(y_average>60)
//	{
//		image_aim_mid=aim_mid-(y_average-50)/20.0*aim_mid;
//	}
//	else
//	image_aim_mid=aim_mid;

//x_average_use����
//#define update_step 15
//if(x_average-x_average_use<=update_step && x_average-x_average_use>=-update_step)

//else
//	{
//		if(x_average-x_average_use>update_step)
//			x_average_use += update_step;
//		if(x_average-x_average_use<-update_step)
//			x_average_use -= update_step;
//	}
		image_fps++;
	}
	
	}
}
	

// **************************** ��ʱ���жϷ������������õĺ��� ****************************
void pit_tim8_hanlder (void)//TIM8��Ҫ�ж�ѭ��    
{
	lastPitch=Pitch;
// **************************** ��ȡICM20602��ԭʼ���� ****************************
	get_icm20602_accdata_spi();														// ��ȡICM20602�Ĳ�����ֵ
	get_icm20602_gyro_spi();														  // ��ȡICM20602�Ĳ�����ֵ
// **************************** ����ŷ���� ****************************
	if(GyroOffset.i)
{                                     //�ȴ���Ư�������
	icm_gyro_x-=GyroOffset.Xdata;
	icm_gyro_y-=GyroOffset.Ydata;
	icm_gyro_z-=GyroOffset.Zdata;
	icm20602_data_change();
  Imu_Update(unit_icm_gyro_x,unit_icm_gyro_y,unit_icm_gyro_z,unit_icm_acc_x,unit_icm_acc_y,unit_icm_acc_z);
	angleRate=Pitch-lastPitch;
// **************************** ��ȡ��������ֵ ****************************
	icm20602_circle++;
	if(icm20602_circle>=4)
	{	
	encoder1_speed =encoder1_speed*0.97 + tim_encoder_get_count_(TIM4)/4.0*0.03;//��
	encoder2_speed =encoder2_speed*0.97 - tim_encoder_get_count_(TIM3)/4.0*0.03;//��
	
	#define encoder_change_max 0.5

	if( encoder2_speed-encoder2_use>encoder_change_max ){encoder2_use+=encoder_change_max;}
	else if( encoder2_speed-encoder2_use<-encoder_change_max ){encoder2_use-=encoder_change_max;}
	else{encoder2_use=encoder2_speed;}
	
	if( encoder1_speed-encoder1_use>encoder_change_max ){encoder1_use+=encoder_change_max;}
	else if( encoder1_speed-encoder1_use<-encoder_change_max ){encoder1_use-=encoder_change_max;}
	else{encoder1_use=encoder1_speed;}	
		
	speed_use=encoder1_use+encoder2_use;
	icm20602_circle=0;
	}
// **************************** ������� ****************************

//distance=distance*0.99+pow(num/300.0,0.1)*100*0.01;
	if( x_average==0 )
		{
					 turn_flag=1;//ת��
					 speedAimMid=speed_3-distance;
					 speedAim=speedAimTurnSet;
						change2_MAX=7;
						change2_MIN=-16;
		}
	 else if( x_average>0 )
	  {
			     turn_flag=0;
					 speedAimMid=speed_1-distance;
					 speedAim=(x_average_use-image_aim_mid -( y_average>80 ? 0: ( 80-y_average)*0.6) );// 
						change2_MAX=7;
						change2_MIN=-16;
		}

		speed_circle++;
		if(speed_circle>=49)
		{					
			change4=change2;
			change2= - pi_control_for_speed(speedERRMid,speedPIDMid,speedAimMid,speed_use);

			float change_rate;
			
			if(turn_flag==0)
				change_rate=2;
			if(turn_flag==1)
				change_rate=1;
			
			if(change2-change4>change_rate)	{ change2=change4+change_rate; }
			if(change2-change4<-change_rate){ change2=change4-change_rate; }			
					
		if(change2> change2_MAX) {change2= change2_MAX;}
		if(change2<change2_MIN){change2=change2_MIN;}       //�ٶȸ�Ԥ���������
		speed_circle=0;
		}

	balance_circle++;
	if(balance_circle==1)
	{

			if(turn_flag==1)
			{
				change3= pid_calculate_4(speedERR,speedPID,speedAim,encoder1_speed-encoder2_speed);			
			}
			if(turn_flag==0)
			{
				change3= pid_calculate_4(directionERR,directionPID,speedAim,encoder1_speed-encoder2_speed);	
			}
			if(turn_flag==2)
			{
				change3=pid_calculate_4(speedERR,speedPID,encoder2_speed/turn_radius,encoder1_speed-encoder2_speed);
			}

#define change3_MAX 400		
		if(change3>change3_MAX){change3=change3_MAX;}
		if(change3<-change3_MAX){change3=-change3_MAX;}       //�ٶ�Ŀ����������
		
	change  = pd_control_for_balance(balanceERR, balancePID,(PitchAim+(change4+(change2- change4)*speed_circle/49))*100,Pitch*100);

	PWMsetLeft  =change+ change3;
	PWMsetRight =change- change3;

	balance_circle=0;
	}
	
#define pwm_MAX 15000		
		if(PWMsetLeft>pwm_MAX){PWMsetLeft=pwm_MAX;}
		if(PWMsetLeft<-pwm_MAX){PWMsetLeft=-pwm_MAX;}       //�ٶ�Ŀ����������
		if(PWMsetRight>pwm_MAX){PWMsetRight=pwm_MAX;}
		if(PWMsetRight<-pwm_MAX){PWMsetRight=-pwm_MAX;}

	state=balance_zero-Pitch;	
	if(state>=30|| state<=-30)
	{drv8701_control(0,0);}
	else
	{drv8701_control(-PWMsetRight,-PWMsetLeft);}
}  
control_fps++;
}

void pit_tim7_hanlder (void)//TIM7��Ҫ�ж�ѭ��    
{
		ips200_showstr(0,200,"fps:");
	  ips200_showfloat(30+calculate_aaa(image_fps),200,image_fps,3,1);
	control_fps=0;
	image_fps=0;
}
// **************************** ���ڽ����жϷ������������õĺ��� ****************************
void uart_order_receive (UART_TypeDef* uart)        //����������պ���
{
	u8 Res;
	Res=uart->RDR;
	orderB =Res-48;
}
// **************************** ��������Ʈ���������������� ****************************
void gyro_OffsetInit(void)     
{
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
		GyroOffset.i=0;
#define data_len 100
    for (int i = 0; i < data_len; i++) 
	{
        GyroOffset.Xdata += icm_gyro_x;
        GyroOffset.Ydata += icm_gyro_y;
        GyroOffset.Zdata += icm_gyro_z;
        systick_delay_ms(1);    // ��� 1Khz
    }

    GyroOffset.Xdata /= data_len;
    GyroOffset.Ydata /= data_len;
    GyroOffset.Zdata /= data_len;
		
		icm20602_data_change();
		q0q1q2q3_init();
		
		GyroOffset.i=1;
}
void q0q1q2q3_init(void)
{
	float norm=invSqrt(unit_icm_acc_x*unit_icm_acc_x+unit_icm_acc_y*unit_icm_acc_y+unit_icm_acc_z*unit_icm_acc_z);
	float ax=unit_icm_acc_x*norm;
	float ay=unit_icm_acc_y*norm;
	float az=unit_icm_acc_z*norm;
	//��ȡһ�μ��ٶȼ�ֵ������Ԫ����ʼ����ʹ����Ԫ������0��ʼ����������ļ��ٿ���ʱ��
	q0= cos(asin(ax)/2)*cos(atan(ay/az)/2);
	q1= sin(asin(ax)/2)*sin(atan(ay/az)/2);
	q2=-sin(asin(ax)/2)*cos(atan(ay/az)/2);
	q3= cos(asin(ax)/2)*sin(atan(ay/az)/2);
}
// **************************** ������ŷ���Ǽ�����غ��� ****************************
static float invSqrt(float x) 		//���ټ��� 1/Sqrt(x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
void Imu_Update(float gx,float gy,float gz,float ax,float ay,float az)
{
	u8 i;
	float vx,vy,vz;							//ʵ���������ٶ�
	float ex,ey,ez;							//�����������
	float norm;
	
 	float q0q0 = q0*q0;
 	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
 	float q1q2 = q1*q2;
 	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	
	if(ax*ay*az == 0)
	return;
	
	//���ٶȼƲ������������򣨻�������ϵ��
	norm = invSqrt(ax*ax + ay*ay + az*az);			
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
//	test_Pitch=asin(-ax)/PI*180;
	test_Pitch=-atan(ax/sqrt(ay*ay+az*az))*180/PI;
	//��Ԫ���Ƴ���ʵ���������򣨻�������ϵ��
	vx = 2*(q1q3 - q0q2);												
  	vy = 2*(q0q1 + q2q3);
  	vz = q0q0 - q1q1 - q2q2 + q3q3;
	
	//������
	ex = (ay*vz - az*vy); 
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	//���������Ϊ���ٶ�
	exInt = exInt + ex * halfT;
	eyInt = eyInt + ey * halfT;
	ezInt = ezInt + ez * halfT;
	
	//���ٶȲ���
	gx = gx + Kp*ex + exInt*Ki;
	gy = gy + Kp*ey + eyInt*Ki;
	gz = gz + Kp*ez + ezInt*Ki;
	
	//������Ԫ��
  	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;	
	
	//��λ����Ԫ��
  	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  	q0 = q0 * norm;
  	q1 = q1 * norm;
  	q2 = q2 * norm;
  	q3 = q3 * norm;
	
	//��Ԫ������ŷ����
//	Yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f;
	Pitch = -asin(2.f * ( +q1q3 -q0q2))* 57.3f;
//	Roll = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f;
}
// **************************** ���ߴ��ڷ��ͺ��� ****************************
void wireless_printf(char *buff1,double data,char *buff2)
{
	seekfree_wireless_send_buff((uint8 *)buff1,strlen(buff1));
	char str[11];
	sprintf(str,"%.7lf",data);
	seekfree_wireless_send_buff((uint8 *)str,10);
	seekfree_wireless_send_buff((uint8 *)buff2,strlen(buff2));
}
// **************************** ��ʱ���жϳ�ʼ�� ****************************
void tim8_interrupt_init_ms(uint32 timer, uint8 preemption_priority, uint8 sub_priority)
{
	if(timer <= 0 || timer > 5000)		return;
	NVIC_InitTypeDef NVIC_InitStructure;														// �ж����ýṹ��
	uint16 freq_div = timer*2-1;																// ����Ԥ��Ƶ
	uint16 period_temp = SystemCoreClock / 2000 - 1;											// �����Զ���װ��ֵ
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM8, ENABLE);							// ʹ��ʱ��

	TIM8->ARR = period_temp;													// װ���Զ���װ��ֵ
	TIM8->PSC = freq_div;														// װ��Ԥ��Ƶ
	TIM8->CR1 |= TIM_CR1_CEN;													// ʹ�ܶ�ʱ��	
	TIM8->DIER |= TIM_DIER_UI;													// ʹ���жϸ���

	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;									// �ж�ѡ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = preemption_priority & 0x03;			// ���������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = sub_priority & 0x03;						// ���������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;												// ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);																// ��ʼ���ж�����

}
void tim7_interrupt_init_ms(uint32 timer, uint8 preemption_priority, uint8 sub_priority)
{
	if(timer <= 0 || timer > 5000)		return;
	NVIC_InitTypeDef NVIC_InitStructure;														// �ж����ýṹ��
	uint16 freq_div = timer*2-1;																// ����Ԥ��Ƶ
	uint16 period_temp = SystemCoreClock / 2000 - 1;											// �����Զ���װ��ֵ
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM7, ENABLE);							// ʹ��ʱ��

	TIM7->ARR = period_temp;													// װ���Զ���װ��ֵ
	TIM7->PSC = freq_div;														// װ��Ԥ��Ƶ
	TIM7->CR1 |= TIM_CR1_CEN;													// ʹ�ܶ�ʱ��	
	TIM7->DIER |= TIM_DIER_UI;													// ʹ���жϸ���

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;									// �ж�ѡ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = preemption_priority & 0x03;			// ���������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = sub_priority & 0x03;						// ���������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;												// ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);																// ��ʼ���ж�����

}
// **************************** ��Ļ��Ϣ��ʾ ****************************
void show_turn(void)
{
	#define first_line 120
  #define second_line 170

				int aaa=0;
				ips200_showstr(first_line,0,"bP:");
					aaa=calculate_aaa(balancePID[0]);
	      ips200_showfloat(second_line+aaa,0,balancePID[0],4,3);
				
				ips200_showstr(first_line,1,"bD:");
					aaa=calculate_aaa(balancePID[2]);
				ips200_showfloat(second_line+aaa,1,balancePID[2],4,3);
	
				ips200_showstr(first_line,2,"sP:");
					aaa=calculate_aaa(speedPIDMid[0]);
	      ips200_showfloat(second_line+aaa,2,speedPIDMid[0],4,3);
				
				ips200_showstr(first_line,3,"sI:");
					aaa=calculate_aaa(speedPIDMid[1]);
				ips200_showfloat(second_line+aaa,3,speedPIDMid[1],4,3);

				ips200_showstr(first_line,4,"sA:");
					aaa=calculate_aaa(speedAimMid);
				ips200_showfloat(second_line+aaa,4,speedAimMid,4,3);				
				
				ips200_showstr(first_line,5,"th:");
					aaa=calculate_aaa(threshhold);
				ips200_showfloat(second_line+aaa,5,threshhold,4,3);
				
				ips200_showstr(first_line,6,"am:");
					aaa=calculate_aaa(image_aim_mid);
				ips200_showfloat(second_line+aaa,6,image_aim_mid,4,3);

				ips200_showstr(first_line,7,"bA:");
					aaa=calculate_aaa(PitchAim);
				ips200_showfloat(second_line+aaa,7,PitchAim,4,3);			

				ips200_showstr(first_line,8,"nm:");
					aaa=calculate_aaa(num_max);
				ips200_showfloat(second_line+aaa,8,num_max,4,3);	

				ips200_showstr(first_line,9,"Tr:");
					aaa=calculate_aaa(turn_radius);
				ips200_showfloat(second_line+aaa,9,turn_radius,4,3);		
	
				ips200_showstr(first_line,10,"ma:");
				aaa=calculate_aaa(mt9v03x_angle);
				ips200_showfloat(second_line+aaa,10,mt9v03x_angle,4,3);	
				
				ips200_showstr(first_line,11,"bf:");
				aaa=calculate_aaa(bounder_flag);
				ips200_showfloat(second_line+aaa,11,bounder_flag,4,3);
				
				ips200_showstr(first_line,12,"mr:");
				aaa=calculate_aaa(mt9v03x_rate);
				ips200_showfloat(second_line+aaa,12,mt9v03x_rate,4,3);

				ips200_showstr(first_line,13,"dp:");
				aaa=calculate_aaa(directionPID[0]);
				ips200_showfloat(second_line+aaa,13,directionPID[0],4,3);
				
				ips200_showstr(first_line,14,"di:");
				aaa=calculate_aaa(directionPID[1]);
				ips200_showfloat(second_line+aaa,14,directionPID[1],4,3);

				ips200_showstr(first_line,15,"dd:");
				aaa=calculate_aaa(directionPID[2]);
				ips200_showfloat(second_line+aaa,15,directionPID[2],4,3);
				
				ips200_showstr(first_line,16,"tp:");
				aaa=calculate_aaa(speedPID[0]);
				ips200_showfloat(second_line+aaa,16,speedPID[0],4,3);

				ips200_showstr(first_line,17,"ti:");
				aaa=calculate_aaa(speedPID[1]);
				ips200_showfloat(second_line+aaa,17,speedPID[1],4,3);
				
				ips200_showstr(first_line,18,"td:");
				aaa=calculate_aaa(speedPID[2]	);
				ips200_showfloat(second_line+aaa,18,speedPID[2]	,4,3);

				ips200_showstr(first_line,19,"ts:");
				aaa=calculate_aaa(speedAimTurnSet	);
				ips200_showfloat(second_line+aaa,19,speedAimTurnSet	,4,3);
}
// **************************** �߽��߼��� ****************************
int calculate_bounder(double mt9v03x_pitch,double K)
{
	#define c 60
	int b=(180-Pitch-mt9v03x_pitch-c)*K;
	return MT9V03X_H-b;
}
// **************************** ������� ****************************
double measure_distance(double K)
{
	double out;
	out=0.48*sin(((MT9V03X_H/2.0-y_average)*K+80)/180*PI)/sin((Pitch+57+(MT9V03X_H/2.0-y_average)*K+80)/180*PI);
	return out;
}

// **************************** �������� ****************************
void key_action(void)
{
		//ʹ�ô˷����ŵ����ڣ�����Ҫʹ��while(1) �ȴ������⴦������Դ�˷�
		//���水��״̬
		key0_state_last = key0_state;
		key1_state_last = key1_state;
		key2_state_last = key2_state;
		key3_state_last = key3_state;
		key4_state_last = key4_state;
		key5_state_last = key5_state;
		key6_state_last = key6_state;
		key7_state_last = key7_state;
	
		//��ȡ��ǰ����״̬
		key0_state = gpio_get(G0);
		key1_state = gpio_get(G1);
		key2_state = gpio_get(G2);
		key3_state = gpio_get(G3);
		key4_state = gpio_get(G4);
		key5_state = gpio_get(G5);
		key6_state = gpio_get(G6);
		key7_state = gpio_get(G7);
		//��⵽��������֮��  ���ſ���λ��־λ
		if(!key0_state && key0_state_last)    key0_flag = 1;		
		if(!key1_state && key1_state_last)    key1_flag = 1;
		if(!key2_state && key2_state_last)    key2_flag = 1;
		if(!key3_state && key3_state_last)    key3_flag = 1;
		if(!key4_state && key4_state_last)    key4_flag = 1;
		if(!key5_state && key5_state_last)    key5_flag = 1;
		if(!key6_state && key6_state_last)    key6_flag = 1;
		if(!key7_state && key7_state_last)    key7_flag = 1;
		
		//��־λ��λ֮�󣬿���ʹ�ñ�־λִ���Լ���Ҫ�����¼�
		if(key0_flag)
		{
			key0_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
			white_square(line,location);
			line++;
			if(line>line_max-1){line=0;} 
			show_turn();
			blue_square(line,location);
		}
		if(key1_flag)
		{
			key1_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
			white_square(line,location);
			line--;
			if(line<0){line=line_max-1;} 
			show_turn();
			blue_square(line,location);
		}
		if(key2_flag)
		{
			key2_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
			white_square(line,location);
			location++;
			if(location>=7){location=0;}			
			show_turn();
			blue_square(line,location);
		}		
		if(key3_flag)   
		{
			key3_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
			white_square(line,location);
			location--;
			if(location<0){location=6;}
			show_turn();
			blue_square(line,location);
		}
		if(key4_flag)   
		{
			key4_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
			add(line,location);
			white_square(line,location);
			show_turn();
			blue_square(line,location);
		}
		if(key5_flag)   
		{
			key5_flag = 0;//ʹ�ð���֮��Ӧ�������־λ
			reduce(line,location);
			white_square(line,location);
			show_turn();
			blue_square(line,location);
		}
}
void add(int line,int location)
{
	if(location<3)
	*dispaly[line]+=pow(10,location-3);
	
	if(location>3)
	*dispaly[line]+=pow(10,location-4);
}
void reduce(int line,int location)
{
	if(location<3)
	*dispaly[line]-=pow(10,location-3);
	
	if(location>3)
	*dispaly[line]-=pow(10,location-4);
}
void blue_square(int line,int location)
{
	int x,y;
	y=line*16+15;
	for(x=170+(-location+6)*8;x<=170+(-location+6)*8+8;x++)
	{
		ips200_drawpoint( x, y ,BLUE);
	}
}
void white_square(int line,int location)
{
	int x,y;
	y=line*16+15;
	for(x=170+(-location+6)*8;x<=170+(-location+6)*8+8;x++)
	{
		ips200_drawpoint( x, y ,WHITE);
	}
}
int calculate_aaa(float target)
{
	if(target<10&&target>-10)
		return 8;
	if( (target>=10&&target<100) || (target<=-10&&target>-100) )
		return 0;
	else
		return -8;
}