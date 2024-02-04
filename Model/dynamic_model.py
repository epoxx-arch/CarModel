"""
车辆运动学状态方程: 连续 和离散实现 
x_dot = A * x + B * u
x_k+1 = A * x_k + B * u_k
"""

import numpy as np 
import random
cos = np.cos 
sin = np.sin 
tan = np.tan 



"""
Vehicle Model:
x = [x,y,theta]
u = [v, sigma]

x = v cos(theta)
y = v sin(theta)
theta = v tan(sigma) / l 

X = [x_dot y_dot theta_dot] = [
        [0 0 -vsin(theta)]
        [0 0 vcos(theta)]
        [0 0 0]
  ]

U = [
    [cos(theta) 0 ]
    [sin(theta) 0 ]
    [tan(sigma)/l v/l/(cos(sigma)**2)]
]
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

X_dot = (X(k+1) - x(k))/T = AX + BU 

X(k+1) = (AT + I)X + BTU

A = AT + I = [
    [1 0 -v * sin(theta) * ts]
    [0 1 v*cos(theta)*ts]
    [0 0  1]
    ]

B = BTU 
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
        return A 

    
    # B : 3 * 2 * N  
    def get_B(self,state,control):
        B = [
            [cos(state[2])*self.Ts,0],
            [sin(state[2] * self.Ts, 0)],
            [control[0] / self.l / cos(state[2])**2]
        ]
        return B 


if __name__ == "__main__":
    init_state = [0,0,0]
    control_seril = [[i, 3 + random.random()] for i in range(100)]
    model = KinematicsContinueModel()
