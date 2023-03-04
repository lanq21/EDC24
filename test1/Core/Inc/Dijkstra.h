#ifndef _DIJKSTRA_H_
#define _DIJKSTRA_H_

#include "stm32f4xx_hal.h"
#define inf 30000
#define MAX_EDGE 800
#define MAX_NODE 250
#define PRIORITY_QUEUE_MAX_SIZE 800

extern uint16_t stack[MAX_NODE];
extern uint16_t stack_top;

void Dijkstra(uint16_t begin_index, uint16_t end_index);
void add(uint16_t a, uint16_t b, uint16_t c);
void clear_edge();


#endif