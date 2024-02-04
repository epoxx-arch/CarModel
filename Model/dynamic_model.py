"""
车辆运动学状态方程: 连续 和离散实现 
x_dot = A * x + B * u
x_k+1 = A * x_k + B * u_k
"""

import numpy as np 
import random
import matplotlib.pyplot as plt 
import math 
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

pi = np.pi 
def normial(angle):
    # 将一个角度转到 - pi ~ pi 
     return math.fmod(angle + math.pi, 2 * math.pi) - math.pi



class KinematicsContinueModel:
    def __init__(self) -> None:
        self.l = 2.5
        self.ts = 0.1

    def forward_simulate(self, state,ref_state,ref_stat2,ref_control, control):
        A = self.get_A(ref_state, ref_control)
        NEW_A = A * self.ts + np.eye(3) 
        NEW_B = self.get_B(ref_state, control) * self.ts
        print(NEW_A,NEW_B)
        X_k1 = NEW_A @ (np.array(state) - np.array(ref_state)) + NEW_B @ (np.array(control) - np.array(ref_control)) + ref_stat2
        # X_k1[2] = normial(X_k1[2])
        return X_k1

    def forward_simulate2(self,state,control):
        new_state = list(state)
        new_state[0] = state[0] + self.ts * control[0] * cos(state[2])
        new_state[1] = state[1] + self.ts * control[0] * sin(state[2])
        new_state[2] = state[2] + self.ts * control[0] * tan(control[1]) / self.l
        return new_state

    def get_A(self,state,control):
        return np.array(
            [
                [0,0,-control[0] * sin(state[2])],
                [0,0,control[0] * cos(state[2])],
                [0,0,0]
            ]
        )
    
    def get_B(self,state,control):
        return np.array(
            [
                [cos(state[2]),0],
                [sin(state[2]),0],
                [tan(state[2])/self.l, control[0] / (self.l * cos(control[1]) ** 2)]
            ]
        )
    

"""
运动学差分模型：
x_k+1 = A * x_k + B * u_k 

A : [x,y,theta]
U : [v,sigma] 

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

    def forward_simulate(self, state, control):
        X_dot = self.get_A(state,control) @ state + self.get_B(state,control) @ control
        state_n = X_dot  + state 
    
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
    

    def forward_simulate(self, state, control):
        ans = [state]
        new_state = state  
        for con in control:
            X_dot = self.get_A(new_state,control) @ new_state + self.get_B(new_state,control) @ control
            new_state = X_dot  + new_state
        ans.append(new_state)
        return np.array(ans) 
    # B : 3 * 2 * N  
    def get_B(self,state,control):
        B = [
            [cos(state[2])*self.Ts,self.z],
            [sin(state[2] * self.Ts, self.z)],
            [control[0] / self.l / cos(state[2])**2, self.Ts * state[2] / self.l / cos(control[1])**2]
        ]
        return B 



def draw(state_s,state_2):
    sign = 0
    for st in [state_2]:
        sign += 1
        st = np.array(st)
        index  = [i  for i in range(len(st))]
        print("数据，",sign,":",st.shape ,":",st)
        # plt.plot(index,st[:,0],label = "x" + str(sign))
        # plt.plot(index,st[:,1],label = "y" + str(sign))
        # plt.plot(index,st[:,2],label = "theta" + str(sign))
        plt.plot(st[:,0],st[:,1],label = "pos" + str(sign))
    #plt.xlim([0,30])
    #plt.ylim([0,30])
    plt.legend()
    plt.show()


if __name__ == "__main__":
    init_state = [0,0,0]
    control_seril = [[3, 0.2] for i in range(100)]
    model_continue = KinematicsContinueModel()

    print("init_s",init_state)
    ans = [init_state]
    ans2 =[init_state]
    state = list(init_state)
    state2 = list(init_state)
    for con in control_seril:
        new_state = list(state2) 
        state2 = model_continue.forward_simulate2(state2,con)
        state = model_continue.forward_simulate(state,new_state ,state2,con,con)
        ans2.append(list(state2))
        ans.append(list(state))

    draw(ans,ans2)
    
    # model_discreate  = KinematicsDiscreteModel()
