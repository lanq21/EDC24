#ifndef PID_H
#define PID_H
#include "stm32f4xx_hal.h"
typedef struct pid{
	float kp,ki,kd;
	float err,iErr,lastErr;
	float tht;
	float goal;
}pid;

void pid_clear(pid *p);
void pid_init(pid *p, float kp, float ki, float kd);
float pid_calculate(pid *p, float current);
#endif
