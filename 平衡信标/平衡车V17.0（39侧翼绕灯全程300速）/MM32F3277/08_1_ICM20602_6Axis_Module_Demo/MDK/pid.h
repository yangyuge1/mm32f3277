#ifndef __PID_H
#define __PID_H

float pid_calculate(float *err,float *PID,float aim,float now);
int pid_calculate_1(float *err,float *PID,float aim,int now);  
float pid_calculate_2(float *err,float *PID,float aim,int now) ;               //位置式
float pi_control_for_speed(float *err,float *PID,int aim,float now);
float pid_calculate_3(float *err,float *PID,float aim,float now);
int pid_calculate_4(float *err,float *PID,float aim,float now);
int pd_control_for_balance(float *err,float *PID,float aim,float Pitch);
int pid_calculate_3_1(float *err,float *PID,float aim,float now);              //位置式

#endif