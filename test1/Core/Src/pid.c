#include "pid.h"
#include "math.h"
#include "stdio.h"
inline void pid_clear(pid *p){
	p->err=p->iErr=p->lastErr=p->tht=p->goal=0;
}

void pid_init(pid *p, float kp, float ki, float kd){
	pid_clear(p);
	p->kp=kp;
	p->ki=ki;
	p->kd=kd;
}

float pid_calculate(pid *p, float current){
	p->err=current-p->goal;
	p->iErr+=p->err;
	/*if(isnan(p->iErr)){
		printf("iErr:%f  err: %f cur: %f  goal: %f\n",p->iErr,p->err,current,p->goal);
	}*/
	//if(p->iErr>1000) p->iErr=1000;
	//if(p->iErr<-1000) p->iErr=-1000;
	p->tht=p->kp*p->err+p->ki*p->iErr+p->kd*(p->err-p->lastErr);
	p->lastErr=p->err;
	return p->tht;
}
