#include "drive.h"
#include "Dijkstra.h"
#include "map.h"
#include "zigbee_edc24.h"
#include "usart.h"
#include "jy62.h"
#include <math.h>
#include "pid.h"

extern pid xPosPid, yPosPid;
extern pid zAnglePid;

float drive_velocity_x_goal;
float drive_velocity_y_goal;
float drive_angle_goal;
uint8_t drive_only_deliver;
enum Drive_State_Type drive_state;
Position_edc24 charge_pile[3] = {{80, 80}, {80, 160}, {160, 120}};
Position_edc24 drive_goal;
Drive_Order drive_order[70];
uint8_t drive_order_total = 0;

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
    float min_distance = 1e30;
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

void Drive_Init()
{
    drive_velocity_x_goal = 0;
    drive_velocity_y_goal = 0;
    drive_angle_goal = 0;
    drive_goal = getVehiclePos();
    drive_only_deliver = 0;
    drive_state = Ready;
}

extern struct vec acc, angle, vel;

int16_t index = 0;
uint8_t Drive_Plan(uint16_t x_goal, uint16_t y_goal)
{
    if (drive_state == Ready)
    {
        Position_edc24 position = getVehiclePos();
        uint16_t begin_index = Get_Nearby_Node(position.x, position.y);
        uint16_t end_index = Get_Nearby_Node(x_goal, y_goal);

        // u1_printf("%d,%d\n%d,%d\n", Node_List[begin_index].x, Node_List[begin_index].y, Node_List[end_index].x, Node_List[end_index].y);

        Dijkstra(begin_index, end_index);
        index = stack_top - 1;

        for (int16_t i = stack_top - 1; i >= 0; --i)
            u1_printf("%d,%d\n", Node_List[stack[i]].x, Node_List[stack[i]].y);
        return 1;
    }
    else
        return 0;
}

void Go_to(uint16_t x_goal, uint16_t y_goal)
{
    if (x_goal != drive_goal.x || y_goal != drive_goal.y)
    {
        Drive_Plan(x_goal, y_goal);
        drive_goal.x = x_goal;
        drive_goal.y = y_goal;
    }
    // drive_velocity_goal = Drive_Speed;
    uint16_t x_step_goal, y_step_goal;
    if (drive_state == Ready)
    {
        // u1_printf("aaa%d,%d\n", position.x, position.y);
        // HAL_Delay(100);
        drive_state = Going;
    }
    else if (drive_state == Going)
    {
        if (index < 0)
            drive_state = Approaching;
        else
        {
            Position_edc24 position = getVehiclePos();
            x_step_goal = Node_List[stack[index]].x;
            y_step_goal = Node_List[stack[index]].y;
            float distance = Get_Distance(position.x, position.y, x_step_goal, y_step_goal);
            if (distance < Distance_Threshold__Next_Node)
                index--;
            else
            {
                // drive_angle_goal = atan2((double)position.x - (double)x_step_goal, (double)y_step_goal - (double)position.y) * 180 / 3.1415926 - 90;
                // drive_velocity_x_goal = Drive_Speed * (float)(x_step_goal - position.x) / distance;
                // drive_velocity_y_goal = Drive_Speed * (float)(y_step_goal - position.y) / distance;
                xPosPid.goal = x_step_goal;
                yPosPid.goal = y_step_goal;
                u1_printf("Going:x:%d, y:%d, goal:(%d, %d), currentAngle=%3.3f, angle=%3.3f\n", position.x, position.y, x_step_goal, y_step_goal, angle.z, drive_angle_goal);
                // if (drive_angle_goal < -180)
                //     drive_angle_goal += 180;
            }
        }
    }
    else if (drive_state == Approaching)
    {
        Position_edc24 position = getVehiclePos();
        float distance = Get_Distance(position.x, position.y, x_goal, y_goal);
        if (distance < Distance_Threshold__Next_Node_For_Approaching)
            drive_state = Ready;
        else
        {
            // drive_angle_goal = atan2((double)position.x - (double)x_goal, (double)y_goal - (double)position.y) * 180 / 3.1415926 - 90;
            // drive_velocity_x_goal = Drive_Speed * (float)(x_goal - position.x) / distance;
            // drive_velocity_y_goal = Drive_Speed * (float)(y_goal - position.y) / distance;
            xPosPid.goal = x_goal;
            yPosPid.goal = y_goal;
            u1_printf("Approaching:x:%d, y:%d, goal:(%d, %d), currentAngle=%3.3f, angle=%3.3f\n", position.x, position.y, x_step_goal, y_step_goal, angle.z, drive_angle_goal);
            // if (drive_angle_goal < -180)
            //    drive_angle_goal += 180;
        }
    }
}

uint8_t Set_Charge_Pile()
{
    static uint8_t i = 0;
    Go_to(charge_pile[i].x, charge_pile[i].y);
    if (drive_state == Approaching)
    {
        setChargingPile();
        charge_pile[i] = getVehiclePos();
        drive_state = Ready;
        i++;
    }
    return i == 3;
}

void Drive_Receive_New_Order()
{
    Order_edc24 new_order = getLatestPendingOrder();
    if (new_order.depPos.x)
    {
        drive_order[drive_order_total].order = new_order;
        drive_order[drive_order_total].state = To_Deliver;
        drive_order[drive_order_total].time = getGameTime();
        drive_order_total++;
    }
}

uint8_t Drive_Change_Order_State_Order_Num = 0;
void Drive_Change_Order_State()
{
    uint16_t id;
    if (Drive_Change_Order_State_Order_Num < getOrderNum())
        id = getOneOrder(getOrderNum() - 1).orderId;
    Drive_Change_Order_State_Order_Num = getOrderNum();
    for (uint8_t i = 0; i < drive_order_total; i++)
    {
        // if (drive_order[i].state == Delivering)
        // {
        //     uint8_t flag = 0;
        //     for (int j = 0; j < order_num; j++)
        //     {
        //         if (GetOneOrder(j).orderID == id)
        //         {
        //             flag = 1;
        //             break;
        //         }
        //     }
        //     if (flag == 0)
        //         drive_order[i].state = Delivered;
        // }
        if (drive_order[i].order.orderId == id)
        {
            drive_order[i].state = Delivering;
            break;
        }
    }
}

uint8_t Drive_First_New_Order(Order_edc24 *order)
{
    uint8_t i = 0;
    for (; i < drive_order_total; i++)
    {
        if (drive_order[i].state == To_Deliver)
        {
            *order = drive_order[i].order;
            return 1;
        }
    }
    order = NULL;
    return 0;
}

uint8_t Drive_Order_To_Deliver(uint32_t *min_cost, Order_edc24 *order)
{
    Position_edc24 vehicle = getVehiclePos();
    uint8_t order_num = getOrderNum();
    if (order_num)
    {
        *min_cost = UINT_LEAST32_MAX;
        uint8_t order_index;
        for (int i = 0; i < order_num; i++)
        {
            Order_edc24 order = getOneOrder(i);
            uint32_t cost = order.timeLimit * Time_Weights_In_Cost +
                            (uint32_t)Get_Distance(vehicle.x, vehicle.y, order.desPos.x, order.desPos.y) * (100 - Time_Weights_In_Cost) * 200;
            if (cost < *min_cost)
            {
                *min_cost = cost;
                order_index = i;
            }
        }
        *order = getOneOrder(order_index);
        return 1;
    }
    else
    {
        order = NULL;
        return 0;
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
    Drive_Receive_New_Order();
    Drive_Change_Order_State();

    uint16_t x_goal = 0, y_goal = 0;
    if (drive_state == Ready)
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
            uint32_t min_cost;
            Order_edc24 order_to_deliver, new_order;
            if (Drive_Order_To_Deliver(&min_cost, &order_to_deliver))
            {
                x_goal = order_to_deliver.desPos.x;
                y_goal = order_to_deliver.desPos.y;
                // if (drive_only_deliver == 0 &&
                //     getOrderNum() < 5 &&
                //     min_cost > Cost_Threshold__Get_New_Order &&
                //     Drive_First_New_Order(&new_order))
                // {
                //     x_goal = new_order.depPos.x;
                //     y_goal = new_order.depPos.y;
                // }
                // else
                // {
                //     x_goal = order_to_deliver.desPos.x;
                //     y_goal = order_to_deliver.desPos.y;
                // }
            }
            else if (Drive_First_New_Order(&new_order))
            {
                x_goal = new_order.depPos.x;
                y_goal = new_order.depPos.y;
            }
        }
    }
    if (x_goal && y_goal)
        Go_to(x_goal, y_goal);
}