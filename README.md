## EDC24

还没想好...

---

2022/10/14

轮子可以转起来，串口还没有验证完，硬件搭建和接线基本完成，现在转到代码阶段。



2022/10/21

潘确然完成了三个轮的 PID 调参，`main.c` 更新到潘确然修改的版本



2022/10/27

完成了轮子转速和角度的 PID 设置



2022/11/27

第一阶段的建图、寻路和开车代码完成，现已整合进项目，建图部分的接口还没有调整



2023/2/27

经过一个假期后的第一次更新，可能遗漏了一部分新补充而未及时上传的代码。`EDC24hw` 部分是第二次作业的 B 机代码。



2023/3/7

轮子换成了普通轮子， `Go_to()` 改为走直线加转向，走直线已经验证了，转弯可能有方向问题，初步写了设置充电桩。



2023/3/17

进度+1，送出第一单。

暂时使用简单逻辑：接一单送一单。路面情况对车的走线影响较大。



2023/3/26

初赛后第一次修改，策略代码基本完整，可行性未知



2023/3/27

目录包含了上位机



2023/3/28

将`Go_to()` 改为 `Go()` ，现通过修改 `drive_goal` 变量决定 `Go` 的目的地



2023/3/30

位移 `pid` 是目前主要问题， `drive.c` 执行过程不稳定，可能会变慢或卡住