#ifndef _DRIVE_H_
#define _DRIVE_H_

#include "stm32f4xx_hal.h"
#include "zigbee_edc24.h"

#define Drive_Speed 2                                   // 速度上限，用于无位置 pid 的 Go_to 方案 2
#define Value_Commission 1                              // 订单估值分数系数
#define Value_Time 1                                    // 订单估值时间系数，剩余时间越短，估值越高
#define Value_Distance 1                                // 订单估值距离系数，距离越短，估值越高
#define Value_Threshold__Change 1000                    // 价值阈值，订单价值与最优价值的差低于此阈值时，不更新最优订单
#define Time_Threshold__Abandon_Order 5                 // 时间阈值，判定是否放弃订单
#define Time_Threshold__Only_Deliver 0                  // 时间阈值，设置是否仅送货
#define Distance_Threshold__Next_Node 20                // 距离阈值，判定是否转到下一个路径点
#define Distance_Threshold__Next_Node_For_Approaching 8 // 距离阈值，判定是否已到达目标点
#define RemainDistance_Threshold__Charge 200            // 剩余里程阈值，判定是否需要充电

enum Drive_State_Type // 状态：就绪；前往下一个中间点；靠近目标点
{
    Ready,
    Going,
    Approaching
};

enum Deliver_State_Simple // （简单逻辑）当前状态
{
    No_Order,
    To_Dep,
    Dep_to_Des
};

enum Drive_Order_State // 订单状态
{
    To_Deliver,
    Delivering,
    Delivered
};

typedef struct
{
    Order_edc24 order;
    enum Drive_Order_State state;
    int32_t receive_time;
} Drive_Order; // 订单列表类型

// Go to 方案 2, 无位置 pid
extern float drive_velocity_x_goal;
extern float drive_velocity_y_goal;
extern float drive_angle_goal;

extern uint8_t drive_only_deliver;        // 标记：时间不足时只送货
extern enum Drive_State_Type drive_state; // 状态：就绪；前往下一个中间点；靠近目标点
extern Position_edc24 charge_pile[3];     // 充电桩位置，在最近的建图节点设置充电桩后更新
extern Drive_Order drive_order[70];       // 订单列表
extern uint8_t drive_order_total;         // 订单列表索引范围

extern Order_edc24 order_simple;                       // （简单逻辑）当前订单
extern enum Deliver_State_Simple deliver_state_simple; // （简单逻辑）当前订单状态

void Go_to(uint16_t x, uint16_t y); // 前往目标点

void Drive_Init(); // 初始化

void Drive_Simple(); // 简单逻辑

void Drive(); // 主逻辑

#endif