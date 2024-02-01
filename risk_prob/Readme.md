# 动态博弈 


# 时序数据

## 1. 时序TTC ： TET / TTT 


$$TTC_i(t) = \left\{
    \begin{align}
        \frac{x_{i-1}(t) - x_i(t) - L_{i-1}}{v_i(t) - v_{i-1}(t)}, & \ \quad v_i(t) - v_{i-1}(t) > 0 \\
        \infty, & \quad \text{otherwise}
    \end{align}
    \right.$$
$$TET_I = \sum_{t=0}^N \sigma_i(t) \tau_{sc}$$

$$\sigma_i(t) = \left\{
\begin{align}
 \ 1, & \ \quad 0 \geq TTC_i(t) \geq TTC^\star \\
    0, & \quad \text{otherwise}
\end{align}
\right.$$

$$TIT_i = \sum_{t=0}^N[TTC^\star - TTC_I(t)] \cdot \tau_{sc}$$
$$\forall 0 \leq t \leq N, \quad 0\leq TTC_i(t) \leq TTC^\star$$

# RSS 
非刹停的，考虑安全容忍度的刹车策略。
$$d^{\prime}_{min} = \max \{ 0, (v_r \rho + \frac{1}{2} a_{max,accel} \rho^2 + \frac{v_r + \rho a_{max,accel}}{2a_{min,brake}}) \} $$

可能存在，时域上后车位置超前的情况。 

1. 计算反应时间 $\rho$ 内的距离改变：
   $$d^{\prime \prime}_{min} = (v_r - v_f)\rho + \frac{a_{max,accel} + a_{max,brake} \rho^2}{2}$$

2. 计算反应时间后的两车之间的距离，时间函数 

    $$d^{\prime \prime \prime}_{min} = (v_r + a_{max,accel}\rho)t_r - \frac{a_{max,brake} t_f^2}{2} - ((v_f - a_{max,brake}\rho)t_f - \frac{a_{max,brake}t_f^2}{2}) $$

# ODD：车辆预期运行环境
安全测试七层架构：
- 自车状态
- 通信信息
- 气候环境
- 交通参与者
- 道路和设施临时改变
- 交通设施
- 道路

# 定义：去预测/预测不准性下的规划风险调控方法。

## Phrase 1：路口场景风险调控





# 记录：

1. 2s跟随规则。


# 目标是什么：
- 两个大车并排，中间空间数据上安全，但是实际上不安全。 
- 倒车：异常交通行为要额外关注 
  - 车头朝向一定程度反应未来车辆运动趋势 
  - 泊车出库：在先外动， 相对位置在不断靠近
  - 掉头：位置在不断逼近 --> 
- 

# 问题：
- 时序数据 -->  构建时序特征