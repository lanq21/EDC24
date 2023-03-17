#ifndef _DRIVE_H_
#define _DRIVE_H_

#include "stm32f4xx_hal.h"
#include "zigbee_edc24.h"

#define Drive_Speed 2
#define Time_Weights_In_Cost 70
#define Cost_Threshold__Get_New_Order 100
#define Time_Threshold__Only_Deliver 0
#define Distance_Threshold__Next_Node 20
#define Distance_Threshold__Next_Node_For_Approaching 8
#define RemainDistance_Threshold__Charge 10000

#define To_Deliver 0
#define Delivering 1
#define Delivered 2

enum Drive_State_Type
{
    Ready,
    Going,
    Approaching
};

enum Deliver_State_Simple
{
    No_Order,
    To_Dep,
    Dep_to_Des
};

typedef struct{
    Order_edc24 order;
    uint8_t state;
    int32_t time;
}Drive_Order;

extern enum Drive_State_Type drive_state;
extern float drive_velocity_x_goal;
extern float drive_velocity_y_goal;
extern float drive_angle_goal;
extern uint8_t drive_only_deliver;
extern Position_edc24 charge_pile[3];
extern Position_edc24 drive_goal;
extern Drive_Order drive_order[70];
extern uint8_t drive_order_total;
extern Order_edc24 order_simple;
extern enum Deliver_State_Simple deliver_state_simple;

float Get_Distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

uint16_t Get_Nearby_Node(uint16_t x, uint16_t y);

void Go_to(uint16_t x_goal, uint16_t y_goal);

uint8_t Set_Charge_Pile();

void Drive_Init();

void Drive_Simple();

void Drive();

#endif