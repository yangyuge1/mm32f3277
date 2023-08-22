#include "pid.h"

extern	float speed_I_MAX;
extern	float I_TIME_LENGTH;    //0.955

// **************************** PID控制相关函数 ****************************
float pid_calculate(float *err,float *PID,float aim,float now)                //增量式
{
	err[2]=err[1];
	err[1]=err[0];
	err[0]=aim-now;
	return (((PID[0]+PID[1]+PID[2])*err[0]+(-PID[0]-2*PID[2])*err[1]+PID[2]*err[2]));
}
int pid_calculate_1(float *err,float *PID,float aim,int now)                //增量式
{
	err[2]=err[1];
	err[1]=err[0];
	err[0]=aim-now;
	return ((int)((PID[0]+PID[1]+PID[2])*err[0]+(-PID[0]-2*PID[2])*err[1]+PID[2]*err[2]));
}
float pid_calculate_2(float *err,float *PID,float aim,int now)                //位置式
{

	err[1]=err[0];
	err[0]=err[1]*0.0+(aim-now)*1.0;//一阶低通滤波
	err[3]+=err[0];	
	err[4]=err[0]-err[1];
#define MAX2 7000
	if(err[3]>=MAX2)
	err[3]=MAX2;
	if(err[3]<=-MAX2)
	err[3]=-MAX2;
	return (PID[0]*err[0]+PID[1]*err[3]+PID[2]*err[4]);
}
float pid_calculate_3(float *err,float *PID,float aim,float now)               //位置式
{
	float err3;
	err[3]+=err[0];
	err[1]= err[0];
	err[0]= aim-now;
	err[4]= err[0]-err[1];
#define I_MAX 500	
	if(err[3]>=I_MAX)
	err[3]=I_MAX;
	if(err[3]<=-I_MAX)
	err[3]=-I_MAX;
	
	return ((PID[0]*err[0]+PID[1]*err[3]+PID[2]*err[4]));
}
int pid_calculate_3_1(float *err,float *PID,float aim,float now)               //位置式
{
	float err3;
	err[3]+=err[0];
	err[1]= err[0];
	err[0]= aim-now;
	err[4]= err[0]-err[1];
#define I_MAX_3_1 2000	
	if(err[3]>=I_MAX_3_1)
	err[3]=I_MAX_3_1;
	if(err[3]<=-I_MAX_3_1)
	err[3]=-I_MAX_3_1;
	
	return ((int)(PID[0]*err[0]+PID[1]*err[3]+PID[2]*err[4]));
}
int pid_calculate_4(float *err,float *PID,float aim,float now)               //位置式
{
	float err3;
	err[3]+=err[0];
	err[1]= err[0];
	err[0]= aim-now;
	err[4]= err[0]-err[1];
	
	if(err[3]>=I_MAX)
	err[3]=I_MAX;
	if(err[3]<=-I_MAX)
	err[3]=-I_MAX;
	
	if(err[0]<=20 &&err[0]>= -20)
	{
		err[3]+=err[0];
	  return ((int)(PID[0]*err[0]+PID[1]*err[3]+PID[2]*err[4]));
	}
  else
	return ((int)(PID[0]*err[0]+PID[2]*err[4]));
}
int pd_control_for_balance(float *err,float *PID,float aim,float Pitch)               //位置式
{
		err[1]= err[0];
	err[0]= aim-Pitch;
	err[4]= err[0]-err[1];	
//	#define Pitch_Err_Max 3000
//	#define Pitch_Err_Min 1000

	
	#define Pitch_Err_I_Max 10000
	
	if(err[3]>=Pitch_Err_I_Max)
	err[3]=Pitch_Err_I_Max;
	if(err[3]<=-Pitch_Err_I_Max)
	err[3]=-Pitch_Err_I_Max;
	
	#define Pitch_Err_Min 300
	
	if(err[0]<=Pitch_Err_Min &&err[0]>= -Pitch_Err_Min)
	{
		err[3]+=err[0];
	  return ((int)(PID[0]*err[0]+PID[1]*err[3]+PID[2]*err[4]));
	}
  else
	return ((int)(PID[0]*err[0]+PID[2]*err[4]));
}
float pi_control_for_speed(float *err,float *PID,int aim,float now)               //位置式
{
	err[1]= err[0];
	err[0]=aim-now;
		err[3]=err[3]*(I_TIME_LENGTH)+err[0];

	err[4]= err[0]-err[1];
//	#define speed_I_MAX 10000
	if(err[3]>=speed_I_MAX)
	err[3]=speed_I_MAX;
	if(err[3]<=-speed_I_MAX)
	err[3]=-speed_I_MAX;
	  return ((PID[0]*err[0]+PID[1]*err[3]+PID[2]*err[4]));
}
