#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "stm32f4xx_hal.h"
#include "zigbee_edc24.h"

#define barrier_num 5
#define num 18

typedef struct
{
    Position_edc24 p;
    int number;
} Node;

struct Edge
{
    int p1;
    int p2;
    int weight;
};

extern Position_edc24* node_list;
extern uint16_t node_list_size;

#endif