#include "drive.h"

#include <math.h>
#include "usart.h"
#include "jy62.h"
#include "zigbee_edc24.h"
#include "pid.h"
#include "map.h"
#include "Dijkstra.h"

// Go to 方案1, 有位置 pid 和角度 pid
extern pid xPosPid, yPosPid, zAnglePid;

// Go to 方案2, 无位置 pid
float drive_velocity_x_goal;
float drive_velocity_y_goal;
float drive_angle_goal;

uint8_t drive_only_deliver;                                        // 标记：时间不足时只送货
enum Drive_State_Type drive_state;                                 // 状态：就绪；前往下一个中间点；靠近目标点
Position_edc24 charge_pile[3] = {{80, 80}, {80, 160}, {160, 120}}; // 充电桩位置，在最近的建图节点设置充电桩后更新
Position_edc24 drive_goal;                                         // 目标点

Drive_Order drive_order[70];   // 订单列表
uint8_t drive_order_total = 0; // 订单列表索引范围

extern Position_edc24 Node_List[num * num]; // "map.h" 节点列表
extern uint16_t node_cnt;                   // "map.h" 节点数

float Get_Distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2) // 计算两点间欧氏距离
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

uint16_t Get_Nearby_Node(uint16_t x, uint16_t y) // 获取最近的节点在 NodeList 的索引
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

// to do
void Drive_Init() // 初始化
{
    // drive_velocity_x_goal = 0;
    // drive_velocity_y_goal = 0;
    // drive_angle_goal = 0;

    drive_goal.x = 0;
    drive_goal.y = 0;
    drive_only_deliver = 0;
    drive_state = Ready;

    // （简单逻辑）初始化
    order_simple.depPos.x = 0;
    order_simple.orderId = -1;
    deliver_state_simple = No_Order;
}

Position_edc24 drive_plan; // 路径规划的目标点
void Drive_Plan()          // 进行一次路径规划
{
    if (drive_plan.x == 0 || drive_plan.y == 0)
        return;
    Position_edc24 position = getVehiclePos();
    uint16_t begin_index = Get_Nearby_Node(position.x, position.y);
    uint16_t end_index = Get_Nearby_Node(drive_plan.x, drive_plan.y);
    Dijkstra(begin_index, end_index);
    u1_printf("drive_plan called, go to(%d, %d)\n", drive_plan.x, drive_plan.y);
    // for (int16_t i = stack_top - 1; i >= 0; --i)
    //     u1_printf("\t node: (%d, %d)\n", Node_List[stack[i]].x, Node_List[stack[i]].y);
}

uint16_t index = 0;                // 当前路径点在 stack 中的索引
extern struct vec acc, angle, vel; // "jy62.h" 加速度、角度、速度，用于串口监测
void Go()                          // 前往目标点
{
    if (drive_goal.x == 0 || drive_goal.y == 0)
        return;
    if ((drive_plan.x != drive_goal.x) || (drive_plan.y != drive_goal.y))
    {
        drive_plan = drive_goal;
        Drive_Plan();        // 进行一次路径规划
        drive_state = Ready; // 暂停上一任务，重置状态
        index = stack_top - 1;
    }
    uint16_t x_step_goal, y_step_goal;

    if (drive_state == Ready)
    {
        drive_state = Going;
        xPosPid.iErr = 0;
        yPosPid.iErr = 0;
    }
    else if (drive_state == Going)
    {
        if (index == 0) // 已到达最后一个路径点
        {
            drive_state = Approaching;
            xPosPid.iErr = 0;
            yPosPid.iErr = 0;
        }
        else
        {
            Position_edc24 position = getVehiclePos();
            x_step_goal = Node_List[stack[index]].x;
            y_step_goal = Node_List[stack[index]].y;
            float distance = Get_Distance(position.x, position.y, x_step_goal, y_step_goal);
            if (distance < Distance_Threshold__Next_Node) // 目标切换到下一个路径点
            {
                xPosPid.iErr = 0;
                yPosPid.iErr = 0;
                index--;
            }
            else // 继续靠近路径点
            {
                // drive_angle_goal = atan2((double)position.x - (double)x_step_goal, (double)y_step_goal - (double)position.y) * 180 / 3.1415926 - 90;
                // if (drive_angle_goal < -180) drive_angle_goal += 180;
                // drive_velocity_x_goal = Drive_Speed * (float)(x_step_goal - position.x) / distance;
                // drive_velocity_y_goal = Drive_Speed * (float)(position.y - y_step_goal) / distance;
                xPosPid.goal = x_step_goal;
                yPosPid.goal = y_step_goal;
                u1_printf("Going: position:(%d,%d), goal:(%d, %d), currentAngle=%3.3f, angle=%3.3f\n", position.x, position.y, x_step_goal, y_step_goal, angle.z, drive_angle_goal);
            }
        }
    }
    else if (drive_state == Approaching)
    {
        Position_edc24 position = getVehiclePos();
        float distance = Get_Distance(position.x, position.y, x_goal, y_goal);
        if (distance < Distance_Threshold__Next_Node_For_Approaching) // 判定为到达
        {
            drive_state = Ready;
            u1_printf("Arrived\n");
        }
        else // 继续靠近目标点
        {
            // drive_angle_goal = atan2((double)position.x - (double)x_goal, (double)y_goal - (double)position.y) * 180 / 3.1415926 - 90;
            // if (drive_angle_goal < -180) drive_angle_goal += 180;
            // drive_velocity_x_goal = Drive_Speed * (float)(x_goal - position.x) / distance;
            // drive_velocity_y_goal = Drive_Speed * (float)(position.y - y_goal) / distance;
            xPosPid.goal = drive_goal.x;
            yPosPid.goal = drive_goal.y;
            u1_printf("Approaching: x:%d, y:%d, goal:(%d, %d), currentAngle=%3.3f, angle=%3.3f\n", position.x, position.y, drive_goal.x, drive_goal.y, angle.z, drive_angle_goal);
        }
    }
}

// 每次调用都会进行一次路径规划的 Go_to()
/*
void Go_to(uint16_t x_goal, uint16_t y_goal)
{
    // drive_velocity_goal = Drive_Speed;
    static int16_t index;
    static uint16_t x_step_goal, y_step_goal;
    if (drive_state == Ready)
    {
        Position_edc24 position = getVehiclePos();
        uint16_t begin_index = Get_Nearby_Node(position.x, position.y);
        uint16_t end_index = Get_Nearby_Node(x_goal, y_goal);

        // u1_printf("%d,%d\n%d,%d\n", Node_List[begin_index].x, Node_List[begin_index].y, Node_List[end_index].x, Node_List[end_index].y);

        Dijkstra(begin_index, end_index);
        index = stack_top - 1;

        u1_printf("aaa%d,%d\n", position.x, position.y);
        // HAL_Delay(100);
        for (int16_t i = stack_top - 1; i >= 0; --i)
        {
            u1_printf("%d,%d\n", Node_List[stack[i]].x, Node_List[stack[i]].y);
        }

        drive_state = Going;
        xPosPid.iErr = yPosPid.iErr = 0;
    }
    else if (drive_state == Going)
    {
        if (index < 0)
        {
            drive_state = Approaching;
            xPosPid.iErr = yPosPid.iErr = 0;
        }
        else
        {
            Position_edc24 position = getVehiclePos();
            x_step_goal = Node_List[stack[index]].x;
            y_step_goal = Node_List[stack[index]].y;
            float distance = Get_Distance(position.x, position.y, x_step_goal, y_step_goal);
            if (distance < Distance_Threshold__Next_Node)
            {
                index--;
                xPosPid.iErr = yPosPid.iErr = 0;
            }
            else
            {
                drive_angle_goal = atan2((double)position.x - (double)x_step_goal, (double)y_step_goal - (double)position.y) * 180 / 3.1415926 - 90;

                drive_velocity_x_goal = Drive_Speed * (float)(x_step_goal - position.x) / distance;
                drive_velocity_y_goal = Drive_Speed * (float)(y_step_goal - position.y) / distance;

                xPosPid.goal = x_step_goal;
                yPosPid.goal = y_step_goal;
                u1_printf("x:%d, y:%d, goal:(%d, %d), currentAngle=%3.3f, angle=%3.3f\n", position.x, position.y, x_step_goal, y_step_goal, angle.z, drive_angle_goal);
                // HAL_Delay(200);

                if (drive_angle_goal < -180)
                    drive_angle_goal += 180;
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
            drive_angle_goal = atan2((double)position.x - (double)x_goal, (double)y_goal - (double)position.y) * 180 / 3.1415926 - 90;
            drive_velocity_x_goal = Drive_Speed * (float)(x_goal - position.x) / distance;
            drive_velocity_y_goal = Drive_Speed * (float)(y_goal - position.y) / distance;

            xPosPid.goal = x_goal;
            yPosPid.goal = y_goal;
            u1_printf("x:%d, y:%d, goal:(%d, %d), currentAngle=%3.3f, angle=%3.3f\n", position.x, position.y, x_step_goal, y_step_goal, angle.z, drive_angle_goal);

            if (drive_angle_goal < -180)
                drive_angle_goal += 180;
        }
    }
}
*/

uint8_t charge_pile_index = 0; // 取 0 1 2，超出时直接返回
void Drive_Set_Charge_Pile()   // 循环调用以设置 3 个充电桩
{
    // if (charge_pile_index < 3)
    // {
    //     deliver_state_simple = To_Dep;
    //     drive_goal = charge_pile[charge_pile_index];
    //     if (drive_state == Approaching)
    //     {
    //         setChargingPile();
    //         u1_printf("set charge pile No.%d\n", charge_pile_index);
    //         charge_pile[charge_pile_index] = getOneOwnPile(charge_pile_index);
    //         drive_state = Ready;
    //         deliver_state_simple = No_Order;
    //         charge_pile_index++;
    //     }
    // }
    if (charge_pile_index < 3 && getGameTime() % 1000 == 0)
    {
        setChargingPile();
        charge_pile[charge_pile_index] = getOneOwnPile(charge_pile_index);
        u1_printf("set charge pile No.%d\n", charge_pile_index);
        charge_pile_index++;
    }
}

void Drive_Receive_New_Order() // 将新订单加入 drive_order 数组
{
    Order_edc24 order = getLatestPendingOrder();
    if (order.depPos.x)
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
                    u1_printf("update order state: order %d is delivering\n", drive_order[j].order.orderId);
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
                    u1_printf("update order state: order %d is delivered\n", drive_order[i].order.orderId);
                }
            }
        }
    }
    // 根据规则描述，超时未送达的订单会被清除，可由订单数减少来探测并处理，所以不需要这一段
    // for (int16_t i = 0; i < drive_order_total; i++)
    // {
    //     if (drive_order[i].state == Delivering)
    //     {
    //         if (getGameTime() - drive_order[i].receive_time > drive_order[i].order.timeLimit + Time_Threshold__Abandon_Order) // 超时未送达
    //         {
    //             drive_order[i].state = Delivered;
    //             u1_printf("update order state: order %d is delivered\n", drive_order[i].order.orderId);
    //         }
    //     }
    // }
    drive_update_order_state_order_num = order_delivering_num;
}

void Drive_Charge() // 充电
{
    uint8_t charging_pile_num = getOwnChargingPileNum();
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
    drive_goal = getOneOwnPile(min_index);
}

// to do
int32_t Drive_Value(Drive_Order drive_order) // 计算订单价值
{
    int32_t value;
    if (drive_order.state == To_Deliver)
        value = Value_Commission * drive_order.order.commission + Value_Time * 1.0 / (drive_order.order.timeLimit) + Value_Distance * 1.0 / Get_Distance(drive_order.order.depPos.x, drive_order.order.depPos.y, getVehiclePos().x, getVehiclePos().y) + Value_Distance * 1.0 / Get_Distance(drive_order.order.desPos.x, drive_order.order.desPos.y, drive_order.order.depPos.x, drive_order.order.depPos.y);
    else if (drive_order.state == Delivering)
        value = Value_Commission * drive_order.order.commission + Value_Time * 1.0 / (drive_order.order.timeLimit - getGameTime() + drive_order.receive_time) + Value_Distance * 1.0 / Get_Distance(drive_order.order.desPos.x, drive_order.order.desPos.y, getVehiclePos().x, getVehiclePos().y);
    else if (drive_order.state == Delivered)
        value = -Value_Threshold__Change;
    return value;
}

int32_t max_value = -Value_Threshold__Change; // 最大价值
uint8_t max_index = 0;                        // 最大价值订单索引
void Drive_Deliver_Order()                    // 接单或送单
{
    if (drive_order[max_index].state == Delivered) // 最大价值订单已送达
        max_value = -Value_Threshold__Change;
    if (drive_state == Ready)
    {
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
                deliver_state_simple = To_Dep;
            }
            else if (drive_order[max_index].state == Delivering)
            {
                drive_goal = drive_order[max_index].order.desPos;
                deliver_state_simple = Dep_to_Des;
            }
        }
        else
            deliver_state_simple = No_Order;
    }
}

int16_t drive_simple_lastOrderID = -1;          // （简单逻辑）上一次订单 id
Order_edc24 drive_simple_order_ready;           // （简单逻辑）准备好的订单
Order_edc24 order_simple;                       // （简单逻辑）当前订单
enum Deliver_State_Simple deliver_state_simple; // （简单逻辑）当前订单状态
void Drive_Simple()                             // 简单逻辑
{
    setChargingPile();
    switch (deliver_state_simple)
    {
    case No_Order:
        if (order_simple.depPos.x) // 有新订单
            deliver_state_simple = To_Dep;
        else
        {
            if (getOrderNum() > 0)
            {
                drive_simple_order_ready = getOneOrder(0);
                if (drive_simple_order_ready.orderId != drive_simple_lastOrderID) // 携带的第一个订单是新订单
                {
                    order_simple = drive_simple_order_ready;
                    deliver_state_simple = Dep_to_Des; // 直接转到递送阶段
                }
            }
            else
                order_simple = getLatestPendingOrder(); // 没有订单，获取最新的订单
            if (order_simple.orderId == drive_simple_lastOrderID || order_simple.depPos.x < 10 || order_simple.depPos.x > 254 - 10 || order_simple.desPos.x < 10 || order_simple.desPos.x > 254 - 10)
            { // 无新订单或者订单不合法
                order_simple.depPos.x = 0;
                deliver_state_simple = No_Order;
            }
            else
                drive_simple_lastOrderID = order_simple.orderId; // 更新上一次订单 id
        }
        break;
    case To_Dep:
        drive_goal = order_simple.depPos;
        if (drive_state == Ready)
            deliver_state_simple = Dep_to_Des;
        break;
    case Dep_to_Des:
        drive_goal = order_simple.desPos;
        if (drive_state == Ready)
        {
            deliver_state_simple = No_Order;
            order_simple.depPos.x = 0;
        }
        break;
    }
}

void Drive() // 主逻辑
{
    if (getGameStatus() == GameStandby)
        return;
    switch (getGameStage())
    {
    case FirstHalf:
        if (getGameTime() + Time_Threshold__Only_Deliver < 60000)
            drive_only_deliver = 0;
        else
            drive_only_deliver = 1;
        Drive_Set_Charge_Pile();
        break;
    case SecondHalf:
        if (getGameTime() + Time_Threshold__Only_Deliver < 180000 + 60000)
            drive_only_deliver = 0;
        else
            drive_only_deliver = 1;
        break;
    default:
        return;
    }
    Drive_Receive_New_Order();
    Drive_Update_Order_State();
    if (getOwnChargingPileNum() && getRemainDist() < RemainDistance_Threshold__Charge)
        Drive_Charge();
    else
        Drive_Deliver_Order();
    Go();
}