#ifndef __ENCODER_H
#define __ENCODER_H
#include "reg_tim.h"
void encoder_init(void);
short int tim_encoder_get_count_(TIM_TypeDef* tim);
#endif
