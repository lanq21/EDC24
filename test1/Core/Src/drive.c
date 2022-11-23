#include "drive.h"
#include "Dijstra.h"
#include "graph.h"
#include "zigbee_edc24.h"

float drive_velocity_x_goal;
float drive_velocity_y_goal;
float drive_angle_goal;
uint8_t only_deliver;

uint32_t Get_Distance(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    return ((x1 < x2) ? (x2 - x1) : (x1 - x2)) + ((y1 < y2) ? (y2 - y1) : (y1 - y2));
}

uint8_t Get_Nearby_Node(uint16_t x, uint16_t y)
{
    uint32_t min_distance = UINT_LEAST32_MAX;
    uint8_t min_index;
    for (uint8_t index = 0; index < node_list_size; index++)
    {
        uint32_t distance = Get_Distance(node_list[index].x, node_list[index].y, x, y);
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
}

void Go_to(uint16_t x_goal, uint16_t y_goal)
{
    Position_edc24 position = getVehiclePos();
    uint8_t begin_index = Get_Nearby_Node(position.x, position.y);
    uint8_t end_index = Get_Nearby_Node(x_goal, y_goal);
    Dijstra(begin_index, end_index);

    uint8_t index = 0;
    uint8_t size = stack_top;
    uint16_t x, y, x_step_goal, y_step_goal;

    while (index < stack_top)
    {
        Position_edc24 position = getVehiclePos();
        x = position.x;
        y = position.y;
        x_step_goal = node_list[stack[index]].x;
        y_step_goal = node_list[stack[index]].y;
        uint32_t distance = Get_Distance(x, y, x_step_goal, y_step_goal);

        if (distance > Distance_Threshold__Next_Node)
        {
            // go to node_list[index]
            drive_velocity_x_goal = Speed * (double)(x_step_goal - x) / distance;
            drive_velocity_y_goal = Speed * (double)(y_step_goal - y) / distance;
        }
        else
            index++;
    }

    uint8_t order_num_before = getOrderNum();
    while (getOrderNum() == order_num_before)
    {
        // go to the destination
        uint32_t distance = Get_Distance(x, y, x_step_goal, y_step_goal);
        if (distance)
        {
            drive_velocity_x_goal = Speed * (double)(x_goal - x) / distance;
            drive_velocity_y_goal = Speed * (double)(y_goal - y) / distance;
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

    uint32_t battery_life = getRemainDist();
    uint8_t charging_pile_num = getOwnChargingPileNum();
    Position_edc24 vehicle = getVehiclePos();

    // charge
    if (charging_pile_num && battery_life < RemainDistance_Threshold__Charge)
    {
        uint32_t min_distance =UINT_LEAST32_MAX;
        uint8_t min_index;
        for (uint8_t i = 0; i < charging_pile_num; i++)
        {
            Position_edc24 pile = getOneOwnPile(i);
            uint32_t distance = Get_Distance(pile.x, pile.y, vehicle.x, vehicle.y);
            if (distance < min_distance)
            {
                min_distance = distance;
                min_index = i;
            }
        }
        Position_edc24 pile = getOneOwnPile(min_index);
        Go_to(pile.x, pile.y);
        HAL_Delay(3000);
    }

    // deliver an order or get new order
    vehicle = getVehiclePos();
    uint8_t order_num = getOrderNum();
    if (order_num)
    {
        uint32_t min_cost = UINT_LEAST32_MAX;
        uint8_t order_index;
        for (int i = 0; i < order_num; i++)
        {
            Order_edc24 order = getOneOrder(i);
            uint32_t cost = order.timeLimit * Time_Weights_In_Cost +
                            Get_Distance(vehicle.x, vehicle.y, order.desPos.x, order.desPos.y) * (100 - Time_Weights_In_Cost) * 200;
            if (cost < min_cost)
            {
                min_cost = cost;
                order_index = i;
            }
        }

        if (order_num < 5 &&
            min_cost > Cost_Threshold__Get_New_Order &&
            only_deliver == 0)
            Go_to(getLatestPendingOrder().depPos.x, getLatestPendingOrder().depPos.y);
        else
            Go_to(getOneOrder(order_index).desPos.x, getOneOrder(order_index).desPos.y);
    }
    else
    {
        Go_to(getLatestPendingOrder().depPos.x, getLatestPendingOrder().depPos.y);
    }
}