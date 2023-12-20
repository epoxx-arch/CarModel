"""
LQR算法的一种简单实现
"""
from typing import Any
import numpy as np 
import 

cf =  155494.663
cr = 155494.663
m = 
lr = 
lf = 
l = 


"""二自由度横向跟踪偏差模型"""
class Model():
    def __init__(self,vx,cr,lr,cf,lf,m) -> None:
        self.A = np.array([
            [0,1,0,0],
            [0,(-cf-cr)/(m*vx), (cf+cr)/m, (cr*lr - cf*lf)/(m*vx)],
            [0,0,0,1],
            [0,(lr*cr-lf*cf)/(l/vx), (lf*cf-lr*cf)/l, (-lr*lr - lf*lf * cf)/(l*vx)]
        ])
        self.B = np.array([0,cf/m,0,lf*cf / l]).T
    
    def __call__(self,curr_state_error, control) -> Any:
        return np.dot(self.A, curr_state_error) + np.dot(self.B,control)

class LQR():
    def __init__(self) -> None:
        pass




if __name__ == 