
import numpy as np 
# 自行车模型 

"""
状态量 x = [x, y, theta]
控制量 u = [v, w]
"""
class BicycleModel:
    def __init__(self) -> None:
        pass


    def get_A(self,x,u): # 参考点车辆模型，给的都是参考点的状态量和控制量
        v = u[0]
        w = u[1]
        theta = x[2]
        A = np.array([[1, 0, -v * np.sin(theta)],
                      [0, 1, v * np.cos(theta)],
                      [0, 0, 1]])
        return A
    
    def get_B(self,x,u):
        v = u[0]
        w = u[1]
        theta = x[2]
        B = np.array([[np.cos(theta), 0],
                      [np.sin(theta), 0],
                      [0, 1]])
        return B
