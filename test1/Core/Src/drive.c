#include "drive.h"
#include "Dijkstra.h"
#include "map.h"
#include "zigbee_edc24.h"
#include "usart.h"
#include <math.h>

float drive_velocity_goal;
float drive_angle_goal;
uint8_t drive_only_deliver;
enum Drive_State_Type drive_state;
Position_edc24 charge_pile[5] = {{80, 80}, {80, 160}, {160, 80}, {160, 160}, {120, 120}};

// extern Position_edc24* node_list;
// extern uint16_t node_list_size;
extern Position_edc24 Node_List[num * num];
extern uint16_t node_cnt;

float Get_Distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

uint16_t Get_Nearby_Node(uint16_t x, uint16_t y)
{
    float min_distance = 1e5;
    uint16_t min_index;
    for (uint16_t index = 1; index <= node_cnt; index++)
    {
        float distance = Get_Distance(Node_List[index].x, Node_List[index].y, x, y);
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
    drive_velocity_goal = 0;
    drive_angle_goal = 0;
    drive_only_deliver = 0;
    drive_state = Ready;
}

void Go_to(uint16_t x_goal, uint16_t y_goal)
{
    drive_velocity_goal = Drive_Speed;
    static uint16_t index;
    static uint16_t x_step_goal, y_step_goal;
    if (drive_state == Ready)
    {
        index = 0;
        // Position_edc24 position = getVehiclePos();
        Position_edc24 position;
        position.x = 0;
        position.y = 120;
        uint16_t begin_index = Get_Nearby_Node(position.x, position.y);
        uint16_t end_index = Get_Nearby_Node(x_goal, y_goal);
        // u1_printf("%d,%d\n%d,%d\n", Node_List[begin_index].x, Node_List[begin_index].y, Node_List[end_index].x, Node_List[end_index].y);
        Dijkstra(begin_index, end_index);
        for (int16_t i = stack_top - 1; i >= 0; --i)
        {
            u1_printf("%d,%d\n", Node_List[stack[i]].x, Node_List[stack[i]].y);
        }
        drive_state = Going;
    }
    else if (drive_state == Going)
    {
        if (index == 0)
            drive_state = Approaching;
        else
        {
            Position_edc24 position = getVehiclePos();
            x_step_goal = Node_List[stack[stack_top - 1 - index]].x;
            y_step_goal = Node_List[stack[stack_top - 1 - index]].y;
            float distance = Get_Distance(position.x, position.y, x_step_goal, y_step_goal);
            if (distance < Distance_Threshold__Next_Node)
                --index;
            else
            {
                drive_angle_goal = atan2(y_step_goal - position.y, x_step_goal - position.x) * 180 / 3.1415926 + 90;
                // 可能要改正负号
                if (drive_angle_goal > 180)
                    drive_angle_goal -= 360;
            }
        }
    }
    else if (drive_state == Approaching)
    {
        Position_edc24 position = getVehiclePos();
        float distance = Get_Distance(position.x, position.y, x_goal, y_goal);
        if (distance < Distance_Threshold__Next_Node)
            drive_state = Ready;
        else
        {
            drive_angle_goal = atan2(y_goal - position.y, x_goal - position.x) * 180 / 3.1415926 + 90;
            // 可能要改正负号
            if (drive_angle_goal > 180)
                drive_angle_goal -= 360;
        }
    }
}

void Set_Charge_Pile()
{
    for (uint8_t i = 0; i < 5; i++)
    {
        Go_to(charge_pile[i].x, charge_pile[i].y);
        if (drive_state == Approaching)
        {
            setChargingPile();
            charge_pile[i] = getVehiclePos();
            drive_state = Ready;
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
            drive_only_deliver = 1;
        break;
    case SecondHalf:
        if (getGameTime() + Time_Threshold__Only_Deliver < getHalfGameDuration() + 60000)
            drive_only_deliver = 1;
        break;
    default:
        return;
    }
    static uint16_t x_goal, y_goal;
    if (drive_state == Ready)
    {
        uint32_t battery_life = getRemainDist();
        uint8_t charging_pile_num = getOwnChargingPileNum();
        if (charging_pile_num && battery_life < RemainDistance_Threshold__Charge)
        {
            // charge
            float min_distance = 1e8;
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
                    drive_only_deliver == 0)
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