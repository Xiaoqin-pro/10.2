# 10.2：三维 UAV 控制点路径规划基础模型

这是按照老师提出的“订单顺序 + 航迹控制点”思路建立的静态三维 UAV 路径规划基础版本，保持胎儿心电项目的简单 MATLAB 风格。

## 当前问题

- 单架无人机、单仓库；
- 20 个静态订单；
- RC101 二维客户位置；
- Gaussian 三维地形；
- 圆柱体障碍物；
- 硬时间窗；
- 订单访问顺序与三维控制点联合优化；
- 暂不包含动态新增订单、取消订单和事件重规划。

## 混合粒子编码

每个粒子由两部分组成：

```text
连续订单优先级 q
+
每个航段的 K 个全局 XYZ 控制点
```

当前设置：

```text
订单数 N = 20
控制点数 K = 2
航段数 = 21（仓库→订单、订单→仓库）
```

订单访问顺序由连续优先级排序得到：

```matlab
[~,order] = sort(q);
route = activeIDs(order);
```

控制点直接作为连续变量使用。整个粒子由标准连续 PSO 的速度和位置公式更新，这一版故意保持老师提出的朴素高维 baseline，不加入局部 d/h 编码、Bezier、B-spline、动态订单或新的多样性算子。

## 适应度

对每个航段构造：

```text
起点 → 控制点1 → 控制点2 → 终点
```

然后计算：

- 三维总飞行距离；
- 地形净空；
- 圆柱障碍物碰撞；
- 最大爬升/下降角；
- 最大转向角；
- 平滑度；
- 硬时间窗和等待时间。

不可行路线使用大罚值，便于标准 PSO 优先寻找可行区域。

## 文件结构

```text
main.m                  静态20订单主程序
CreateModel.m           地形、障碍物、订单和时间窗
PSO.m                   连续混合粒子 PSO
Fitness.m               控制点折线和时间窗评价
InitialControlPoints.m  生成控制点初始插值
DecodeParticle.m        将粒子解码成航段折线
PlotSolution.m          路线和全部订单绘图
data/rc101.txt          RC101 二维客户数据
results/                运行结果
```

## 运行

```matlab
cd('D:\111\Desktop\噜噜\10.2');
main
```

结果：

```text
results/main_result.mat
results/baseline_route_3D.png
results/all_orders_route_3D.png
results/baseline_convergence.png
```

当前版本是老师思路的第一版静态 baseline。初始化时先从粒子 q 解码订单路线，再按同一条路线生成控制点；XYZ 控制点在粒子中的展开顺序固定为 x,y,z 逐点排列；pitch 使用带正负号的角度计算，硬约束使用绝对值，平滑度使用 pitch 变化。后续先观察控制点维度增加后标准 PSO 的表现，再决定是否设计改进算法。
