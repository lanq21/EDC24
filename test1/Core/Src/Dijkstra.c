#include "Dijkstra.h"

struct Edge {
	uint16_t to, weight, next;
}e[MAX_EDGE];
uint16_t head[MAX_EDGE];

char vis[MAX_NODE];

uint16_t dij2prque[MAX_NODE];

uint16_t node_cnt = 0;

struct Dij_Node {
	uint16_t x;
	uint16_t dis;
};

struct priority_queue {
	struct Dij_Node nodes[PRIORITY_QUEUE_MAX_SIZE];
	uint16_t cnt;
};

struct priority_queue djq;

uint16_t minTree[MAX_NODE];//zuiDuanLuJingShu
uint16_t stack[MAX_NODE];
uint16_t stack_top = 0;

/*priority_queue*/
void priority_queue_init(struct priority_queue* pq) {
	pq->cnt = 1;
}

void priority_queue_swap_node(struct priority_queue* pq, uint16_t x, uint16_t y) {
	struct Dij_Node tmp = pq->nodes[x];
	uint16_t temp;
	pq->nodes[x] = pq->nodes[y];
	pq->nodes[y] = tmp;

	temp = dij2prque[pq->nodes[x].x];
	dij2prque[pq->nodes[x].x] = dij2prque[pq->nodes[y].x];
	dij2prque[pq->nodes[y].x] = temp;
}

void priority_queue_up(struct priority_queue* pq, uint16_t cur) {
	uint16_t fa = cur / 2;
	if (cur == 1) return;
	if (pq->nodes[cur].dis < pq->nodes[fa].dis) {
		priority_queue_swap_node(pq, fa, cur);
		priority_queue_up(pq, fa);
	}
}

void priority_queue_update_weight(struct priority_queue* pq, uint16_t x, uint16_t newDis) {
	uint16_t nodeNum = dij2prque[x];
	pq->nodes[nodeNum].dis = newDis;
	priority_queue_up(pq, nodeNum);
}
void priority_queue_insert(struct priority_queue* pq, uint16_t x, uint16_t dis) {
	pq->nodes[pq->cnt].x = x;
	pq->nodes[pq->cnt].dis = dis;
	dij2prque[x] = pq->cnt;
	priority_queue_up(pq, pq->cnt);
	pq->cnt++;
	return;
}
void priority_queue_down(struct priority_queue* pq, uint16_t x) {
	uint16_t l, r;
	if (x * 2 + 1 < pq->cnt) {
		l = pq->nodes[x * 2].dis;
		r = pq->nodes[x * 2 + 1].dis;
		if (l <= r && pq->nodes[x].dis > l) {
			priority_queue_swap_node(pq, x, x * 2);
			priority_queue_down(pq, x * 2);
			return;
		}
		else if (l > r && pq->nodes[x].dis > r) {
			priority_queue_swap_node(pq, x, x * 2 + 1);
			priority_queue_down(pq, x * 2 + 1);
			return;
		}
	}
	else if (x * 2 < pq->cnt && pq->nodes[x].dis > pq->nodes[x * 2].dis) {
		priority_queue_swap_node(pq, x, x * 2);
	}
	return;
}
void priority_queue_delete(struct priority_queue* pq) {
	pq->cnt--;
	priority_queue_swap_node(pq, 1, pq->cnt);
	priority_queue_down(pq, 1);
}

/*dijkstra*/
void Dijkstra(uint16_t s, uint16_t t) {
	priority_queue_init(&djq);

	for (uint16_t i = 1; i <= node_cnt; ++i) vis[i] = 0;
	for (uint16_t i = 1; i <= node_cnt; ++i) minTree[i] = 0;

	for (uint16_t i = 1; i <= node_cnt; ++i) {
		if (i == s) priority_queue_insert(&djq, i, 0);
		else priority_queue_insert(&djq, i, inf);
	}
	djq.nodes[dij2prque[s]].dis = 0;

	int tmp = 0;
	while (djq.cnt>1 && tmp < node_cnt) {
		int minx = djq.nodes[1].x;
		int mindis = djq.nodes[1].dis;
		if (mindis == inf)break;

		priority_queue_delete(&djq);

		if (vis[minx]) continue;
		tmp++;
		vis[minx] = 1;
		for (int i = head[minx]; i; i = e[i].next) {
			if (djq.nodes[dij2prque[minx]].dis + e[i].weight < djq.nodes[dij2prque[e[i].to]].dis) {
				priority_queue_update_weight(&djq, e[i].to, djq.nodes[dij2prque[minx]].dis + e[i].weight);
				minTree[e[i].to] = minx;
			}
		}
	}
	uint16_t ptr = t;
	stack_top = 0;
	while (ptr != s&&ptr!=0) {
		stack[stack_top++] = ptr;
		ptr = minTree[ptr];
	}
	stack[stack_top++] = s;
	return;
}

uint16_t edge_cnt = 1;
void add(uint16_t a, uint16_t b, uint16_t c) {
	e[edge_cnt].weight = c;
	e[edge_cnt].to = b;
	e[edge_cnt].next = head[a];
	head[a] = edge_cnt++;
}

void clear_edge(){
	edge_cnt = 1;
	for(int i = 0; i<MAX_NODE; ++i){
		head[i] = 0;
	}
}