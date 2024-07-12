

import matplotlib.pyplot as plt

import numpy as np 

# 一个双连杆机构，求解其运动学方程，并绘制其轨迹
# 机构参数
l1 = 2
l2 = 1

# 机构的初始状态
theta1 = 0
theta2 = 3

# 机构的终止状态
theta1_end = 2 * np.pi
theta2_end = 2 * np.pi

# 机构的运动学方程

def forward_kinematics(theta1,theta2):
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x,y

def inverse_kinematics(x,y):

    # 机构的运动学方程
    # x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    # y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)

    # 机构的逆运动学方程
    # x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    # y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)

    # 两式相减
    # x - l1 * np.cos(theta1) = l2 * np.cos(theta1 + theta2)
    # y - l1 * np.sin(theta1) = l2 * np.sin(theta1 + theta2)

    # 两式平方相加
    # (x - l1 * np.cos(theta1))**2 + (y - l1 * np.sin(theta1))**2 = l2**2 * (np.cos(theta1 + theta2)**2 + np.sin(theta1 + theta2)**2)
    # (x - l1 * np.cos(theta1))**2 + (y - l1 * np.sin(theta1))**2 = l2**2

    # 两式相减
    # (x - l1 * np.cos(theta1))**2 + (y - l1 * np.sin(theta1))**2 - l2**2 = 0

    # 令
    # A = (x - l1 * np.cos(theta1))**2 + (y - l1 * np.sin(theta1))**2 - l2**2
    # A = 0

    # 由于机构的逆运动学方程有两个解，所以需要分别求解
    # 1. theta1 + theta2 = arccos(A / l2)
    # 2. theta1 + theta2 = -arccos(A / l2)

    A = (x - l1 * np.cos(theta1))**2 + (y - l1 * np.sin(theta1))**2 - l2**2
    theta2_1 = np.arccos(A / l2)
    theta2_2 = -np.arccos(A / l2)

    theta1_1 = np.arctan2(y - l1 * np.sin(theta1),x - l1 * np.cos(theta1)) - theta2_1
    theta1_2 = np.arctan2(y - l1 * np.sin(theta1),x - l1 * np.cos(theta1)) - theta2_2

    return theta1_1,theta2_1,theta1_2,theta2_2

# 以动画的形式绘制机构的轨迹
fig,ax = plt.subplots()
ax.set_xlim(-3,3)
ax.set_ylim(-3,3)
line, = ax.plot([],[],'o-')

def init():
    line.set_data([],[])
    return line,

def animate(i):
    theta1 = theta1_end * i / 100
    theta2 = theta2_end * i / 100
    x,y = forward_kinematics(theta1,theta2)
    line.set_data(x,y)
    return line,

from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig,animate,frames=100,interval=100,blit=True)
plt.show()
