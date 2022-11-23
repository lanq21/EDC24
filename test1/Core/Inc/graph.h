#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "stm32f4xx_hal.h"

struct Node
{
    uint16_t x;
    uint16_t y;
};

extern struct Node* node_list; 
extern uint8_t node_list_size;

#endif