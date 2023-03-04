
#include<stdio.h>
//#include<iostream>
#include<stdint.h>
#include<stdlib.h>
#include "usart.h"
#include "zigbee_edc24.h"
#include "Dijkstra.h"
#include "map.h"

#define barrier_num 5
#define num 18

//using namespace ::std;


Position_edc24 wall[8][4];  //四个围墙用8个矩形表示
Node node[num - 1][num - 1];
uint8_t isOK[num - 1][num - 1];

Barrier_edc24 barrier[13];   //五个障碍物
int16_t x_temp[num];//临时数组
int16_t y_temp[num];//临时数组
int16_t x_order[num];//最终使用的数组，数组元素不重复且按从小到大的顺序排列
int16_t y_order[num];
int16_t x_order_num = 0, y_order_num = 0;//最终建立的有序数组中的非0数目
//int16_t node_num = 0;
//int16_t edge_num = 0;
extern uint16_t node_cnt;

Position_edc24 Node_List[num * num];

void Init_Barrier()//围墙位置坐标初始化
{
    //围墙也视作障碍，对于围墙各个点的坐标值
    barrier[5].pos_1.x = 38;
    barrier[5].pos_1.y = 214;
    barrier[5].pos_2.x = 107;
    barrier[5].pos_2.y = 216;
    barrier[6].pos_1.x = 38;
    barrier[6].pos_1.y = 38;
    barrier[6].pos_2.x = 107;
    barrier[6].pos_2.y = 40;
    barrier[7].pos_1.x = 38;
    barrier[7].pos_1.y = 38;
    barrier[7].pos_2.x = 40;
    barrier[7].pos_2.y = 107;
    barrier[8].pos_1.x = 147;
    barrier[8].pos_1.y = 38;
    barrier[8].pos_2.x = 216;
    barrier[8].pos_2.y = 40;
    barrier[9].pos_1.x = 214;
    barrier[9].pos_1.y = 38;
    barrier[9].pos_2.x = 216;
    barrier[9].pos_2.y = 107;
    barrier[10].pos_1.x = 214;
    barrier[10].pos_1.y = 147;
    barrier[10].pos_2.x = 216;
    barrier[10].pos_2.y = 216;
    barrier[11].pos_1.x = 147;
    barrier[11].pos_1.y = 214;
    barrier[11].pos_2.x = 216;
    barrier[11].pos_2.y = 216;
    barrier[12].pos_1.x = 38;
    barrier[12].pos_1.y = 147;
    barrier[12].pos_2.x = 40;
    barrier[12].pos_2.y = 216;
}

int cmp(const void* a, const void* b)//排序函数，从小到大排序
{
    return *(int16_t*)a - *(int16_t*)b;
}
void Init_Coordinate() //障碍物位置坐标初始化
{
    uint16_t i = 0;
    uint16_t num_0=0;
    for (i = 0; i < barrier_num; i++)//从上位机获取生成的障碍物位置信息
    {
        barrier[i]=getOneBarrier(i);
			//u1_printf("%dok:%d,%d\n", i, barrier[i].pos_1.x, barrier[i].pos_1.y);
        //这里需要加代码
        //有一个要求，是各个障碍物是单独发送的，不过这个情况肯定成立
        //x_temp[3~12]是x方向的各个点的横坐标值,y_temp同理;

    }
   
    x_temp[0] = 1;
    x_temp[1] = 38;
    x_temp[2] = 40;
    x_temp[13] = 107;
    x_temp[14] = 147;
    x_temp[15] = 214;
    x_temp[16] = 216;
    x_temp[17] = 254;
    y_temp[0] = 1;
    y_temp[1] = 38;
    y_temp[2] = 40;
    y_temp[13] = 107;
    y_temp[14] = 147;
    y_temp[15] = 214;
    y_temp[16] = 216;
    y_temp[17] = 254;
    //判断左上角和右下角两个顶点，移入barrier数组
    for (i = 3; i < 13; i=i+2)
    {
      x_temp[i] = barrier[(i - 3)/2].pos_1.x;
      x_temp[i+1] = barrier[(i - 3)/2].pos_2.x;
    }
    for (i = 3; i < 13; i=i+2)
    {
       y_temp[i] = barrier[(i - 3)/2].pos_1.y;
       y_temp[i+1] = barrier[(i - 3)/2].pos_2.y;
    }
    //进行排序
    qsort(x_temp, num, sizeof(int16_t), cmp);
    qsort(y_temp, num, sizeof(int16_t), cmp);
    if (x_temp[0] != 0) {
        x_order[x_order_num] = x_temp[0]; x_order_num++;
    }
    if (y_temp[0] != 0) {
        y_order[y_order_num] = y_temp[0]; y_order_num++;
    }

    for (i = 1; i < num; i++)
    {
        
        if ((x_temp[i] != 0)&&x_temp[i]!=x_temp[i-1])
        {
            x_order[x_order_num] = x_temp[i];
            x_order_num++;
        }

    }

    for (i = 1; i < num; i++)
    {
        if ((y_temp[i] != 0)&&(y_temp[i]!=y_temp[i-1]))
        {
            y_order[y_order_num] = y_temp[i];
            y_order_num++;
        }
    }
}

int Judgement(int16_t node_x, int16_t node_y)
{

    int m = 0;
    for (m = 0; m < 13; m++)
    {
        if (node_x > barrier[m].pos_1.x && node_x < barrier[m].pos_2.x)
        {
            if (node_y > barrier[m].pos_1.y && node_y < barrier[m].pos_2.y)
            {
                return 0;
                break;
            }
        }
    }
    return 1;
}

void BuildMap(){
	Init_Barrier();
  Init_Coordinate();
	
	clear_edge();
	
  uint16_t i = 0, j = 0;
	
	//edge_num = 0;
	node_cnt = 1;
	
  int temp_node_x = 0, temp_node_y = 0;
	//建交叉点
	for (i = 0; i < y_order_num-1; i++) 
	{
			for (j = 0; j < x_order_num-1; j++)
			{
					temp_node_x = (x_order[j] + x_order[j + 1]) / 2;
					temp_node_y = (y_order[i] + y_order[i + 1]) / 2;
					if(Judgement(temp_node_x, temp_node_y)){
						node[j][i].number = node_cnt++;
						node[j][i].p.x = temp_node_x;
						node[j][i].p.y = temp_node_y;
						Node_List[node[j][i].number].x = temp_node_x;
						Node_List[node[j][i].number].y = temp_node_y;
						isOK[j][i] = 1;
					}else{
						isOK[j][i] = 0;
					}
			}
	}
	//建边并进行判断
	for (i = 0; i < y_order_num - 1; i++)
	{
			for (j = 0; j < x_order_num - 2; j++)
			{
					if ((node[j][i].p.x!=0)&&(node[j][i].p.y!=0)&&
							node[j+1][i].p.x!=0&&node[j+1][i].p.y!=0&&
							isOK[j][i] && isOK[j + 1][i] &&
							Judgement((node[j][i].p.x + node[j + 1][i].p.x) / 2, (node[j][i].p.y + node[j + 1][i].p.y) / 2))
					{
							//edge[edge_num].p1 = node[j][i].number;
							//edge[edge_num].p2 = node[j + 1][i].number;
							//edge[edge_num].weight = node[j + 1][i].p.x - node[j][i].p.x;
							//edge_num++;
							add(node[j][i].number, node[j + 1][i].number, node[j + 1][i].p.x - node[j][i].p.x);
							add(node[j + 1][i].number, node[j][i].number, node[j + 1][i].p.x - node[j][i].p.x);
						//u1_printf("%d,%d,%d,%d\n", node[j][i].p.x, node[j][i].p.y, node[j+1][i].p.x, node[j+1][i].p.y);
						//HAL_Delay(10);
					}
			}
	}
	for (j = 0; j < x_order_num - 1; j++)
	{
			for (i = 0; i < y_order_num - 2; i++)
			{
					if ((node[j][i].p.x != 0) && (node[j][i].p.y != 0) &&
							node[j][i+1].p.x != 0 && node[j][i+1].p.y != 0 &&
							isOK[j][i] && isOK[j][i + 1] && 
							Judgement((node[j][i].p.x + node[j][i + 1].p.x)/2, (node[j][i].p.y + node[j][i + 1].p.y)/2))
					{
							//edge[edge_num].p1 = node[j][i].number;
							//edge[edge_num].p2 = node[j][i + 1].number;
							//edge[edge_num].weight = node[j][i + 1].p.y - node[j][i].p.y;
							//edge_num++;
							add(node[j][i].number, node[j][i + 1].number, node[j][i + 1].p.y - node[j][i].p.y);
							add(node[j][i + 1].number, node[j][i].number, node[j][i + 1].p.y - node[j][i].p.y);
						//u1_printf("%d,%d,%d,%d\n", node[j][i].p.x, node[j][i].p.y, node[j][i+1].p.x, node[j][i+1].p.y);
						//HAL_Delay(10);
					}
			}
	}
	//u1_printf("ok:%d\n", node_cnt);
}
