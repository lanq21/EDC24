#ifndef _DIJSTRA_H_
#define _DIJSTRA_H_

#include "stm32f4xx_hal.h"
#define inf 2147483647
#define MAX_EDGE 2000
#define MAX_NODE 1000
#define PRIORITY_QUEUE_MAX_SIZE 1000

extern int stack[MAX_NODE];
extern int stack_top;

void Dijstra(int begin_index, int end_index);

#endif