#include "drive.h"
#include "Dijkstra.h"
#include "map.h"
#include "zigbee_edc24.h"
#include <math.h>

float drive_velocity_x_goal;
float drive_velocity_y_goal;
float drive_angle_goal;
uint8_t only_deliver;
enum Drive_State_Type Drive_State;

float Get_Distance(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

uint16_t Get_Nearby_Node(uint16_t x, uint16_t y)
{
    float min_distance = 1e30;
    uint16_t min_index;
    for (uint16_t index = 0; index < node_list_size; index++)
    {
        float distance = Get_Distance(node_list[index].x, node_list[index].y, x, y);
        if (distance < min_distance)
        {
            min_distance = distance;
            min_index = index;
        }
    }
    return min_index;
}

void Set_Angle_Goal()
{

    // if (zAnglePid.goal < -180.0f)
    //     zAnglePid.goal = 180;
    // if (zAnglePid.goal > 180.0f)
    //     zAnglePid.goal = -180;
}

void Drive_Init()
{
    drive_velocity_x_goal = 0;
    drive_velocity_y_goal = 0;
    drive_angle_goal = 0;
    only_deliver = 0;
    Drive_State = Ready;
}

void Go_to(uint16_t x_goal, uint16_t y_goal)
{
    static uint16_t index;
    static uint16_t x_step_goal, y_step_goal;
    if (Drive_State == Ready)
    {
        index = 0;
        Position_edc24 position = getVehiclePos();
        uint16_t begin_index = Get_Nearby_Node(position.x, position.y);
        uint16_t end_index = Get_Nearby_Node(x_goal, y_goal);
        Dijkstra(begin_index, end_index);
        Drive_State = Going;
    }
    else if (Drive_State == Going)
    {
        if (index == stack_top)
            Drive_State = Approaching;
        else
        {
            Position_edc24 position = getVehiclePos();
            x_step_goal = node_list[stack[stack_top - 1 - index]].x;
            y_step_goal = node_list[stack[stack_top - 1 - index]].y;
            float distance = Get_Distance(position.x, position.y, x_step_goal, y_step_goal);
            if (distance < Distance_Threshold__Next_Node)
                index++;
            else
            {
                drive_velocity_x_goal = Speed * (float)(x_step_goal - position.x) / distance;
                drive_velocity_y_goal = Speed * (float)(y_step_goal - position.y) / distance;
            }
        }
    }
    else if (Drive_State == Approaching)
    {
        Position_edc24 position = getVehiclePos();
        float distance = Get_Distance(position.x, position.y, x_goal, y_goal);
        if (distance < Distance_Threshold__Next_Node)
            Drive_State = Ready;
        else
        {
            drive_velocity_x_goal = Speed * (float)(x_goal - position.x) / distance;
            drive_velocity_y_goal = Speed * (float)(y_goal - position.y) / distance;
        }
    }
}

void Drive()
{
    if (getGameStatus() == GameStandby)
        return;
    switch (getGameStage())
    {
    case FirstHalf:
        if (getGameTime() + Time_Threshold__Only_Deliver < getHalfGameDuration())
            only_deliver = 1;
        break;
    case SecondHalf:
        if (getGameTime() + Time_Threshold__Only_Deliver < getHalfGameDuration() + 60000)
            only_deliver = 1;
        break;
    default:
        return;
    }
    static uint16_t x_goal, y_goal;
    if (Drive_State == Ready)
    {
        uint32_t battery_life = getRemainDist();
        uint8_t charging_pile_num = getOwnChargingPileNum();
        if (charging_pile_num && battery_life < RemainDistance_Threshold__Charge)
        {
            // charge
            float min_distance = 1e30;
            uint8_t min_index;
            Position_edc24 vehicle = getVehiclePos();
            for (uint8_t i = 0; i < charging_pile_num; i++)
            {
                Position_edc24 pile = getOneOwnPile(i);
                float distance = Get_Distance(pile.x, pile.y, vehicle.x, vehicle.y);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    min_index = i;
                }
            }
            Position_edc24 pile = getOneOwnPile(min_index);
            x_goal = pile.x;
            y_goal = pile.y;
        }
        else
        {
            // deliver an order or get new order
            Position_edc24 vehicle = getVehiclePos();
            uint8_t order_num = getOrderNum();
            if (order_num)
            {
                uint32_t min_cost = UINT_LEAST32_MAX;
                uint8_t order_index;
                for (int i = 0; i < order_num; i++)
                {
                    Order_edc24 order = getOneOrder(i);
                    uint32_t cost = order.timeLimit * Time_Weights_In_Cost +
                                    (uint32_t)Get_Distance(vehicle.x, vehicle.y, order.desPos.x, order.desPos.y) * (100 - Time_Weights_In_Cost) * 200;
                    if (cost < min_cost)
                    {
                        min_cost = cost;
                        order_index = i;
                    }
                }
                if (order_num < 5 &&
                    min_cost > Cost_Threshold__Get_New_Order &&
                    only_deliver == 0)
                {
                    x_goal = getLatestPendingOrder().depPos.x;
                    y_goal = getLatestPendingOrder().depPos.y;
                }
                else
                {
                    x_goal = getOneOrder(order_index).desPos.x;
                    y_goal = getOneOrder(order_index).desPos.y;
                }
            }
            else
            {
                x_goal = getLatestPendingOrder().depPos.x;
                y_goal = getLatestPendingOrder().depPos.y;
            }
        }
    }
    Go_to(x_goal, y_goal);
}