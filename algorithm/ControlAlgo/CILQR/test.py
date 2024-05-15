from obstacle import Obstacle
from CILQR import CILQR 



def main():

    init_state = []
    obstacle_info = Obstacle()
    cilqr = CILQR(init_state) 
    # 单帧模式：
    res = cilqr.solver(obstacle_info)
    cilqr.draw() 
