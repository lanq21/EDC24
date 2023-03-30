#include "drive.h"

#include <math.h>
#include "usart.h"
#include "jy62.h"
#include "zigbee_edc24.h"
#include "pid.h"
#include "map.h"
#include "Dijkstra.h"

// Go to 方案1, 有位置 pid 和角度 pid
extern pid xPosPid, yPosPid;

uint8_t Drive_Only_Deliver;        // 标记：时间不足时只送货
enum Drive_State_Type drive_state; // 状态：就绪；前往下一个中间点；靠近目标点
Position_edc24 drive_goal;         // 目标点
int16_t drive_order_delivering_count = 0;
Position_edc24 drive_charge_pile[3];

Drive_Order drive_order[70];   // 订单列表
uint8_t drive_order_total = 0; // 订单列表索引范围

extern Position_edc24 Node_List[num * num]; // "map.h" 节点列表
extern uint16_t node_cnt;                   // "map.h" 节点数

float Get_Distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2) // 计算两点间欧氏距离
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

uint16_t Get_Nearby_Node(uint16_t x, uint16_t y)
{
    float min_distance = 1e20;
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

void Drive_Init() // 初始化
{
    drive_goal.x = 0;
    drive_goal.y = 0;
    Drive_Only_Deliver = 0;
    drive_state = To_Set_Charge_Pile;
    drive_charge_pile[0].x = 128;
    drive_charge_pile[0].y = 60;
    drive_charge_pile[1].x = 128;
    drive_charge_pile[1].y = 128;
    drive_charge_pile[2].x = 128;
    drive_charge_pile[2].y = 180;
}

void Rush()
{
    Position_edc24 pos = getVehiclePos();
    if (Get_Distance(pos.x, pos.y, drive_goal.x, drive_goal.y) < Distance_Threshold__Arrived)
    {
        if (drive_state == To_Charge && getRemainDist() > 3000)
        {
            drive_state = Ready;
            xPosPid.iErr = 0;
            yPosPid.iErr = 0;
            u1_printf("charge completed\n");
        }
        else if (drive_state == To_Dep)
        {
            drive_state = Ready;
            xPosPid.iErr = 0;
            yPosPid.iErr = 0;
            u1_printf("To_Dep completed\n");
        }
        else if (drive_state == Dep_to_Des)
        {
            drive_state = Ready;
            xPosPid.iErr = 0;
            yPosPid.iErr = 0;
            u1_printf("Dep_to_Des completed\n");
        }
        else if (drive_state == To_Set_Charge_Pile)
        {
            drive_state = Setting_Charge_Pile;
            xPosPid.iErr = 0;
            yPosPid.iErr = 0;
            u1_printf("setting charge pile\n");
        }
    }
    else
    {
        xPosPid.goal = drive_goal.x;
        yPosPid.goal = drive_goal.y;
    }
}

uint8_t drive_charge_pile_index = 0;
void Drive_Set_Charge_Pile() // 循环调用以设置 3 个充电桩
{
    if (drive_charge_pile_index < 3)
    {
        if (drive_state == Setting_Charge_Pile)
        {
            setChargingPile();
            HAL_Delay(50);
            if (getOwnChargingPileNum() > drive_charge_pile_index)
            {
                drive_state = To_Set_Charge_Pile;
                if (drive_charge_pile_index == 2)
                    drive_state = Ready;
                drive_charge_pile_index++;
                u1_printf("go (%d,%d) to set charge pile\n", drive_goal.x, drive_goal.y);
            }
        }
        else
            drive_goal = Node_List[Get_Nearby_Node(drive_charge_pile[drive_charge_pile_index].x, drive_charge_pile[drive_charge_pile_index].y)];
    }
}

void Drive_Receive_New_Order() // 将新订单加入 drive_order 数组
{
    Order_edc24 order = getLatestPendingOrder();
    if (order.depPos.x > 5 && order.depPos.x < 254 - 5 && order.desPos.x > 5 && order.desPos.x < 254 - 5)
    {
        if (drive_order_total == 0)
        {
            drive_order[0].order = order;
            drive_order[0].state = To_Deliver;
            u1_printf("new order: %d\n", order.orderId);
            drive_order_total++;
        }
        else if (order.orderId != drive_order[drive_order_total - 1].order.orderId)
        {
            for (int16_t i = 0; i < drive_order_total; i++)
            {
                if (order.orderId == drive_order[i].order.orderId)
                    return;
            }
            drive_order[drive_order_total].order = order;
            drive_order[drive_order_total].state = To_Deliver;
            u1_printf("new order: %d\n", order.orderId);
            drive_order_total++;
        }
    }
}

uint8_t drive_update_order_state_order_num = 0; // 上一次更新订单状态时的携带订单数
int16_t delivering_order_id[5];                 // 正在运送的订单 id
void Drive_Update_Order_State()                 // 更新订单状态
{
    // 携带订单数 +1 ：To_Deliver -> Delivering
    // 携带订单数 -1 ：Delivering -> Delivered
    // 超时未送达：Delivering -> Delivered

    uint8_t order_delivering_num = getOrderNum(); // 正在运送的订单数
    for (int16_t i = 0; i < order_delivering_num; i++)
        delivering_order_id[i] = getOneOrder(i).orderId;

    if (order_delivering_num > drive_update_order_state_order_num) // 携带订单数 +1
    {
        for (int16_t i = drive_update_order_state_order_num; i < order_delivering_num; i++)
        {
            for (int16_t j = drive_order_total - 1; j >= 0; j--)
            {
                if (drive_order[j].order.orderId == delivering_order_id[i])
                {
                    drive_order[j].state = Delivering;
                    drive_order[j].receive_time = getGameTime();
                    u1_printf("order %d update: To_Deliver -> Delivering\n", drive_order[j].order.orderId);
                    break;
                }
            }
        }
    }
    else if (order_delivering_num < drive_update_order_state_order_num) // 携带订单数 -1
    {
        for (int16_t i = drive_order_total - 1; i >= 0; i--)
        {
            if (drive_order[i].state == Delivering)
            {
                int16_t j = 0;
                for (; j < order_delivering_num; j++)
                {
                    if (drive_order[i].order.orderId == delivering_order_id[j])
                        break;
                }
                if (j == order_delivering_num)
                {
                    drive_order[i].state = Delivered;
                    u1_printf("order %d update: Delivering -> Delivered\n", drive_order[j].order.orderId);
                }
            }
        }
    }
    drive_update_order_state_order_num = order_delivering_num;
}

void Drive_Update_Order_State_Simple()
{
    uint8_t order_num = getOrderNum();
    for (int16_t i = 0; i < drive_order_total; i++)
    {
        if (drive_order[i].state == To_Deliver)
        {
            for (int16_t j = 0; j < order_num; j++)
            {
                if (drive_order[i].order.orderId == getOneOrder(j).orderId)
                {
                    drive_order[i].state = Delivering;
                    drive_order_delivering_count++;
                    drive_order[i].receive_time = getGameTime();
                    u1_printf("order %d update: To_Deliver -> Delivering\n", drive_order[i].order.orderId);
                    break;
                }
            }
        }
    }
}

void Drive_Charge() // 充电
{
    uint8_t charging_pile_num = getOwnChargingPileNum();
    float min_distance = 1e20;
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
    drive_state = To_Charge;
    drive_goal = getOneOwnPile(min_index);
}

int32_t Drive_Value(Drive_Order drive_order) // 计算订单价值
{
    float value;
    if (drive_order.state == To_Deliver)
        value = Value_Commission * drive_order.order.commission + Value_Time * 1000.0 / (drive_order.order.timeLimit) + Value_Distance * 1000.0 / Get_Distance(drive_order.order.depPos.x, drive_order.order.depPos.y, getVehiclePos().x, getVehiclePos().y) + Value_Distance * 1000.0 / Get_Distance(drive_order.order.desPos.x, drive_order.order.desPos.y, drive_order.order.depPos.x, drive_order.order.depPos.y);
    else if (drive_order.state == Delivering)
        value = Value_Commission * drive_order.order.commission + Value_Time * 1000.0 / (drive_order.order.timeLimit - getGameTime() + drive_order.receive_time) + Value_Distance * 1000.0 / Get_Distance(drive_order.order.desPos.x, drive_order.order.desPos.y, getVehiclePos().x, getVehiclePos().y);
    else if (drive_order.state == Delivered)
        value = -Value_Threshold__Change;
    return value;
}

float max_value = -Value_Threshold__Change; // 最大价值
uint8_t max_index = 0;                      // 最大价值订单索引
void Drive_Deliver_Order()                  // 接单或送单
{
    if (drive_state == Ready)
    {
        max_value = -Value_Threshold__Change;
        for (int16_t i = 0; i < drive_order_total; i++)
        {
            int32_t value = Drive_Value(drive_order[i]);
            if (value > max_value + Value_Threshold__Change)
            {
                max_value = value;
                max_index = i;
            }
        }
        if (max_value > 0) // 有订单可执行
        {
            if (drive_order[max_index].state == To_Deliver)
            {
                drive_goal = drive_order[max_index].order.depPos;
                drive_state = To_Dep;
            }
            else if (drive_order[max_index].state == Delivering)
            {
                drive_goal = drive_order[max_index].order.desPos;
                drive_state = Dep_to_Des;
            }
        }
    }
}

uint8_t drive_deliver = 0; // 0: 接单 1: 送单
void Drive_Deliver_Order_Simple()
{
    if (drive_state == Ready)
    {
        uint8_t order_num = getOrderNum();
        if (order_num == 0 && drive_order_delivering_count == drive_order_total)
            return;
        else if (order_num == 0 && drive_order_delivering_count != drive_order_total && drive_deliver == 1)
        {
            drive_deliver = 0;
            u1_printf("stage update: To_Dep\n");
        }
        else if (order_num != 0 && drive_order_delivering_count == drive_order_total && drive_deliver == 0)
        {
            drive_deliver = 1;
            u1_printf("stage update: Dep_to_Des\n");
        }
        else
        {
            if (Drive_Only_Deliver == 1)
            {
                drive_deliver = 1;
                u1_printf("stage update: Dep_to_Des\n");
            }
            else
            {
                if (order_num == 0 && drive_deliver == 1)
                {
                    drive_deliver = 0;
                    u1_printf("stage update: To_Dep\n");
                }
                else if (order_num >= 4 && drive_deliver == 0)
                {
                    drive_deliver = 1;
                    u1_printf("stage update: Dep_to_Des\n");
                }
            }
        }
        if (drive_deliver)
        {
            int16_t min_index = 0;
            float min_distance = 1e20;
            Position_edc24 pos = getVehiclePos();
            for (int16_t i = 0; i < order_num; i++)
            {
                Order_edc24 order = getOneOrder(i);
                float distance = Get_Distance(order.desPos.x, order.desPos.y, pos.x, pos.y);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    min_index = i;
                }
            }
            if (min_distance < 1e19)
            {
                Order_edc24 order_to_deliver = getOneOrder(min_index);
                drive_state = Dep_to_Des;
                drive_goal = order_to_deliver.desPos;
                u1_printf("Dep_to_Des: (%d, %d), order %d\n", drive_goal.x, drive_goal.y, order_to_deliver.orderId);
            }
        }
        else
        {
            int16_t min_index = 0;
            float min_distance = 1e20;
            for (int16_t i = 0; i < drive_order_total; i++)
            {
                if (drive_order[i].state == To_Deliver)
                {
                    Position_edc24 pos = getVehiclePos();
                    float distance = Get_Distance(drive_order[i].order.depPos.x, drive_order[i].order.depPos.y, pos.x, pos.y);
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        min_index = i;
                    }
                }
            }
            if (min_distance < 1e19)
            {
                drive_state = To_Dep;
                drive_goal = drive_order[min_index].order.depPos;
                u1_printf("To_Dep: (%d, %d), order %d\n", drive_goal.x, drive_goal.y, drive_order[min_index].order.orderId);
                return;
            }
        }
    }
}

void Drive() // 主逻辑
{
    if (getGameStatus() == GameStandby)
        return;
    switch (getGameStage())
    {
    case FirstHalf:
        Drive_Only_Deliver = (getGameTime() + Time_Threshold__Only_Deliver > 60000);
        Drive_Set_Charge_Pile();
        break;
    case SecondHalf:
        Drive_Only_Deliver = (getGameTime() + Time_Threshold__Only_Deliver > 180000 + 60000);
        break;
    default:
        return;
    }
    Drive_Receive_New_Order();
    Drive_Update_Order_State_Simple();
    if (getOwnChargingPileNum() && getRemainDist() < RemainDistance_Threshold__Charge)
        Drive_Charge();
    else
        Drive_Deliver_Order_Simple();
    Rush();
}