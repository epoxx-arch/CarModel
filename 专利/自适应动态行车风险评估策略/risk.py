"""
1. 均值
2. 方差
3. 给定PATH
4. 计算风险分布函数
"""
from operator import index
import numpy as np 
import matplotlib.pyplot as plt



class Config:
    one_dimension_variance = 6
    x_car = 0
    y_car = 0.0
    theta = 1
    v = 10.0
    t = 3.0
    variance_coeff = 10



def mean(s,config):
    # return np.exp(-s**2 / 2 / config.one_dimension_variance**2) 
    return 0
def variance(s,config):
    return 1/(2*np.pi*config.v) * np.exp(-s**2 / 2 / config.v**2) * config.variance_coeff**3


def path_point(x_last,y_last,theta,v,t):

    x = x_last + v * np.cos(theta) * t
    y = y_last + v * np.sin(theta) * t
    return x,y


def path_generate(config:Config):
    x_car = config.x_car
    y_car = config.y_car
    theta = config.theta
    v = config.v
    t = config.t

    delta_t = 0.1 
    path = []
    accumulate_s = 0
    x_last,y_last = x_car,y_car 
    for i in range(int(t / delta_t)):
        x,y = path_point(x_last,y_last,theta,v,delta_t)
        path.append((x,y))
        x_last,y_last = x,y
    
    for i in range(len(path) - 1):
        accumulate_s += ((path[i][0] - path[i+1][0])**2 + (path[i][1] - path[i+1][1])**2)**0.5

    return path,accumulate_s


def point_to_path_dis(x,y,path):
    min_dis = 1000000
    index = -1 
    for i in range(len(path)):
        dis = ((x - path[i][0])**2 + (y - path[i][1])**2)**0.5
        if dis < min_dis:
            min_dis = dis
            index = i

    return min_dis,index

def s(index,path):
    accumulate_s = 0
    for i in range(index):
        accumulate_s += ((path[i][0] - path[i+1][0])**2 + (path[i][1] - path[i+1][1])**2)**0.5
    return accumulate_s

def risk(x,y,config,path,path_accumulate_s):
    # 二维高斯，
    min_dis,index = point_to_path_dis(x,y,path)
    _s =path_accumulate_s - s(index,path)
    _mean = mean(_s,config)
    _variance = variance(_s,config)
    # print("mean:",_mean)
    # print("variance:",_variance)
    return 1/ (np.pi *2 *_variance)**0.5  * np.exp(-(min_dis - _mean) ** 2 / 2 / _variance ** 2)


def plot_one_dimension_gaussian(mean,variance):
    import matplotlib.pyplot as plt
    import numpy as np
    x = np.linspace(-10,20,100)
    y = np.ndarray(x.shape)
    y = 1/ (2 * np.pi * variance) ** 0.5 * np.exp(-(x - mean) ** 2 / 2 / variance)
    plt.plot(x,y)
    # plt.show()


def main():
    import matplotlib.pyplot as plt
    import numpy as np
    config = Config()
    x = np.linspace(0,10,100)
    y = np.linspace(-10,10,100)
    X,Y = np.meshgrid(x,y)
    Z = np.ndarray(X.shape)
    path,path_accumulate_s = path_generate(config)
    print(path)
    for i in range(len(X)):
        for j in range(len(X[i])):
            Z[i][j] = risk(X[i][j],Y[i][j],config,path,path_accumulate_s)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(X,Y,Z)

    # ax = fig.add_subplot(111)
    # 热力图绘制 
    # ax.imshow(Z)
    # ax.plot(np.array(path)[:,0]*10,np.array(path)[:,1]*10)
    plt.show()


main()
# for mean in range(0,10):
#     for variance in range(1,10):
#         plot_one_dimension_gaussian(0,10)

# plt.legend()
# plt.show()

