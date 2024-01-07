# -*- coding:utf-8 -*-
"""
二维平面问题， 构建一个扇形，扇形时刻决定棱长，


1. 单个障碍物的能量场构建 

- 每个时间总能量固定--> 基于速度表征该时刻的总能量。
- 每个时间点能量分布不固定，分布由历史状态更新而来
- 帧之间能量转移--> 如果从一个时间过渡到下一个时间， 能量转移过程中，历史预测的当前帧的能力存在一定的粘滞性，即转换并不是立刻生效的，
而是存在一定的震荡，考虑PID控制器的思想，可以考虑引入积分项，即历史状态的能量分布对当前状态的能量分布有一定的影响，这样可以保证能量分布的连续性，而不是突变。


 r^2 = x^2 + y^2，  

 
2. 常规能量场的构建通常基于自车与它车的位置信息、速度信息来构建，主要考虑的是自车与它车的相对位置，以及它车的速度信息，这样可以构建一个能量场，然后在能量场中进行路径规划。但这种能量场
"""


#初步描述程一个圆形障碍物
class Obstacle(object):
    def __init__(self, x, y, r,heading):
        self.x = x
        self.y = y
        self.r = r
        self.heading = heading
        self.pred_tra = self.pred_tra()
    


# 场是一个扇形，同时有一个维度表示具体的能量值。 
# (theta_0 - theta, theta_0 + theta) 范围内， l = f(t)  t越大， f(t) 越大， 同时单点能量越小 
# e = g(f(t),theta) 
# 约束， \sum e dl = E_0 


class EnergyField(object):
    def __init__(self, obs:Obstacle, dt):
        self.dt = dt
        self.obstacle = obs
        self.energy = 0
        self.energy_list = 

    
    def cal_energy_field(self, pred_tra,):
