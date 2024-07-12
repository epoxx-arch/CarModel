from calendar import c
from dataclasses import dataclass
from logging import config
import math

from click import FLOAT
import numpy as np
import matplotlib.pyplot as plt
tan = math.tan


# 车辆转向半径 

@dataclass
class Config:
    steer_angle:float = 0.0
    L:float = 2.0
    car_width_number:float = 1.8 / 4
    tla:float = 3.5
    theta:float = 0.1
    x_c:float = 5.0 
    y_c:float = 0.0 
    x_car:float = 0.0 
    y_car:float = 3.0 
    m:float = 0.001  # 弧度补偿 
    p = 0.0064
    k1 = 0
    k2 = 1.3823 
    c = 0.5
    v = 10

def r_car(L,theta):
    return L / (tan(theta) + 1e-6)

def normalize_to_0_pi(angle):
    angle = angle % (2 * np.pi)
    return ((angle + np.pi) % (2 * np.pi)) - np.pi

# 弧长公式 
def s(x,y,config):
    # 已知起点、终点，圆弧线，求弧长

    return 

def cal_a(s,config:Config):
    return config.p * (s - config.v * config.tla) ** 2 

def sigma(s,config:Config):
    k11 = (config.m + config.k1 * np.fabs(config.steer_angle)) * s + config.c
    k22 = (config.m + config.k2 * np.fabs(config.steer_angle)) * s + config.c
    return k11 ,k22 

def z(x,y,config:Config, v):
    c = ((x - config.x_c) ** 2 + (y - config.y_c) ** 2) ** 0.5 - r_car(config.L,config.theta)
    print("r_car",r_car(config.L,config.theta))
    _s =s(x,y,config)
    print("s",_s)
    k11,k22 = sigma(_s,config)
    print(k11,k22)
    dd1 = (c / k11)**2 / 2
    dd2  = (c / k22)**2 / 2
    print("dd",dd1,dd2)
    _a = cal_a(_s,config)
    print("a",_a)
    return _a * np.exp(-dd1),_a * np.exp(-dd2)


def main():
    import matplotlib.pyplot as plt
    import numpy as np
    config = Config()
    x = np.linspace(-10,10,100)
    y = np.linspace(-10,10,100)
    X,Y = np.meshgrid(x,y)
    r = r_car(config.L,config.theta)
    print("r",r)
    x_cen = config.x_c 
    y_cen =(r**2 -   (config.x_c - config.x_car) ** 2) ** 0.5 - config.y_car
    config.x_c = x_cen
    config.y_c = y_cen

    Z1,Z2 = z(X,Y,config,10)
    Z = np.ndarray(Z1.shape)
    for i in range(len(Z1)):
        for j in range(len(Z1[i])):
            if ((X[i][j] - config.x_c)**2 +( Y[i][j] - config.y_c)**2) > r_car(config.L,config.theta) ** 2:
                Z[i][j] = Z2[i][j]
            else:
                Z[i][j] = Z1[i][j]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(X,Y,Z)
    print(Z)
    plt.show()

main()
# standard_gaussian(5,5,1,1,0.8,0.8)