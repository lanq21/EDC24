#include "map.h"

Position_edc24 *node_list;
uint16_t node_list_size;

Position_edc24 wall[8][4]; //四个围墙用8个矩形表示
Node map_node[num - 1][num - 1];
Barrier_edc24 barrier[13]; //五个障碍物
struct Edge edge[256];
int x_temp[num];  //临时数组
int y_temp[num];  //临时数组
int x_order[num]; //最终使用的数组，数组元素不重复且按从小到大的顺序排列
int y_order[num];
uint16_t x_order_num = 0, y_order_num = 0; //最终建立的有序数组中的非0数目
int edge_num = 0;

void Init_Barrier()
{
    //围墙也视作障碍
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

int cmp(const void *a, const void *b)
{
    return *(int *)a - *(int *)b; //从小到大排序
}

void Init_Coordinate() //初始化x坐标与y坐标
{
    int i = 0;
    int num_0 = 0;
    for (i = 0; i < barrier_num; i++)
    {
        // barrier[i]=getgetOneBarrier(barrierNo)
        // scanf("%u%u%u%u", &(barrier[i].pos_1.x), &(barrier[i].pos_1.y), &(barrier[i].pos_2.x), &(barrier[i].pos_2.y));
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
    for (i = 3; i < 13; i = i + 2)
    {
        x_temp[i] = barrier[(i - 3) / 2].pos_1.x;
        x_temp[i + 1] = barrier[(i - 3) / 2].pos_2.x;
    }
    for (i = 3; i < 13; i = i + 2)
    {
        y_temp[i] = barrier[(i - 3) / 2].pos_1.y;
        y_temp[i + 1] = barrier[(i - 3) / 2].pos_2.y;
    }
    qsort(x_temp, num, sizeof(int), cmp);
    qsort(y_temp, num, sizeof(int), cmp);
    if (x_temp[0] != 0)
    {
        x_order[x_order_num] = x_temp[0];
        x_order_num++;
    }
    if (y_temp[0] != 0)
    {
        y_order[y_order_num] = y_temp[0];
        y_order_num++;
    }

    for (i = 1; i < num; i++)
    {

        if ((x_temp[i] != 0) && x_temp[i] != x_temp[i - 1])
        {
            x_order[x_order_num] = x_temp[i];
            x_order_num++;
        }
    }

    for (i = 1; i < num; i++)
    {
        if ((y_temp[i] != 0) && (y_temp[i] != y_temp[i - 1]))
        {
            y_order[y_order_num] = y_temp[i];
            y_order_num++;
        }
    }
}

int Judgement(int node_x, int node_y)
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

void Map()
{
    int i = 0, j = 0;
    int temp_node_x = 0, temp_node_y = 0;
    //建交叉点
    for (i = 0; i < y_order_num - 1; i++)
    {
        for (j = 0; j < x_order_num - 1; j++)
        {
            temp_node_x = (x_order[j] + x_order[j + 1]) / 2;
            temp_node_y = (y_order[i] + y_order[i + 1]) / 2;
            map_node[j][i].number = 100 * j + i + 1; //一个散列函数
            map_node[j][i].p.x = temp_node_x;
            map_node[j][i].p.y = temp_node_y;
        }
    }
    //建边并进行判断
    for (i = 0; i < y_order_num - 2; i++)
    {
        for (j = 0; j < x_order_num - 2; j++)
        {
            if (Judgement((map_node[j][i].p.x + map_node[j + 1][i].p.x) / 2, (map_node[j][i].p.y + map_node[j + 1][i].p.y) / 2) && Judgement(map_node[j][i].p.x, map_node[j][i].p.y) && Judgement(map_node[j + 1][i].p.x, map_node[j + 1][i].p.y))
            {
                edge[edge_num].p1 = map_node[j][i].number;
                edge[edge_num].p2 = map_node[j + 1][i].number;
                edge[edge_num].weight = map_node[j + 1][i].p.x - map_node[j][i].p.x;
                edge_num++;
            }
        }
    }
    for (j = 0; j < x_order_num - 2; j++)
    {
        for (i = 0; i < y_order_num - 2; i++)
        {
            if (Judgement((map_node[j][i].p.x + map_node[j][i + 1].p.x) / 2, (map_node[j][i].p.y + map_node[j][i + 1].p.y) / 2) && Judgement(map_node[j][i].p.x, map_node[j][i].p.y) && Judgement(map_node[j][i + 1].p.x, map_node[j][i + 1].p.y))
            {
                edge[edge_num].p1 = map_node[j][i].number;
                edge[edge_num].p2 = map_node[j][i + 1].number;
                edge[edge_num].weight = map_node[j][i + 1].p.y - map_node[j][i].p.y;
                edge_num++;
            }
        }
    }
}