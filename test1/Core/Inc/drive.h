#ifndef _DRIVE_H_
#define _DRIVE_H_

#include "stm32f4xx_hal.h"

#define Speed 10
#define Time_Weights_In_Cost 70
#define Cost_Threshold__Get_New_Order 100
#define Time_Threshold__Only_Deliver 200
#define Distance_Threshold__Next_Node 3
#define RemainDistance_Threshold__Charge 10000

enum Drive_State_Type
{
    Ready,
    Going,
    Approaching
};

extern enum Drive_State_Type Drive_State;
extern float drive_velocity_x_goal;
extern float drive_velocity_y_goal;
extern float drive_angle_goal;
extern uint8_t only_deliver;

float Get_Distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

uint16_t Get_Nearby_Node(uint16_t x, uint16_t y);

void Set_Angle_Goal();

void Go_to(uint16_t x_goal, uint16_t y_goal);

void Drive_Init();

void Drive();

#endif