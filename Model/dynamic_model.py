"""
车辆运动学状态方程: 连续 和离散实现 
x_dot = A * x + B * u


x_k+1 = A * x_k + B * u_k

"""

import numpy as np 

cos = np.cos 
sin = np.sin 
tan = np.tan 



"""
Vehicle Model:
x = [x,y,theta]
u = [v, w]
"""
class KinematicsContinueModel:
    def __init__(self) -> None:
        self.l = 2.5

    def get_A(self,state,control):
        return np.array(
            [
                [0,0,-control[1] * sin(state[2])],
                [0,0,control[1] * cos(state[2])],
                [0,0,0]
            ]
        )
    
    def get_B(self,state,control):
        return np.array(
            [
                [cos(state[2]),0],
                [sin(state[2]),0],
                [tan(state[2])/self.l,control[0] / (self.l * cos(state[2]) ** 2)]
            ]
        )
    

"""
运动学差分模型：
x_k+1 = A * x_k + B * u_k 

A : [x,y,theta]
U : [v,w] 

离散结果，没给一个状态量 + 一个控制量 --> 解出一个 A, B 

但是，一个轨迹是一个 状态量 + 控制量序列，因此，需要 多一个维度表示轨迹长度 

state: 3 * n 
control: 2 * n 
"""
class KinematicsDiscreteModel:
    def __init__(self) -> None:
        self.l = 2.5
        self.Ts = 0.1
        self.nums = 100
        self.o = np.ones((self.nums))
        self.z = np.zeros((self.nums))
    
    # A : 3 * 3 * N
    def get_A(self,state,control):

        A = np.array(
            [
                [self.o, self.z, -control[0] * sin(state[2]) * self.Ts],
                [self.z, self.o, control[0] * cos(state[2]) * self.Ts],
                [self.z, self.z, self.o]
            ]
        )

    
    # B : 3 * 2 * N  
    def get_B(self,state,control):
        B = 

