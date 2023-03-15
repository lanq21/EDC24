#ifndef JY62_H
#define JY62_H
#include "stm32f4xx_hal.h"
#define JY62_BUFFSIZE 200

struct vec{
	float x,y,z;
};

void calibrate(void);
void setVertical(void);
void setHorizontal(void);
void initAngle(void);
void jy62_init(UART_HandleTypeDef *huart);

#endif
