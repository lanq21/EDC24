#ifndef _DRIVE_H_
#define _DRIVE_H_

#include "stm32f4xx_hal.h"
#include "zigbee_edc24.h"

#define Drive_Speed 20
#define Time_Weights_In_Cost 70
#define Cost_Threshold__Get_New_Order 100
#define Time_Threshold__Only_Deliver 200
#define Distance_Threshold__Next_Node 35
#define RemainDistance_Threshold__Charge 10000

enum Drive_State_Type
{
    Ready,
    Going,
    Approaching
};

extern enum Drive_State_Type drive_state;
extern float drive_velocity_goal;
extern float drive_angle_goal;
extern uint8_t drive_only_deliver;
extern Position_edc24 charge_pile[5];

float Get_Distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

uint16_t Get_Nearby_Node(uint16_t x, uint16_t y);

void Go_to(uint16_t x_goal, uint16_t y_goal);

void Drive_Init();

void Drive();

#endif