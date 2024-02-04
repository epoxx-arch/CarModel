import matplotlib.pyplot as plt
from pylab import *
import math
plt.rcParams['axes.unicode_minus'] = False
mpl.rcParams['font.sans-serif'] = ['SimHei']


def GeneralTrans(s_e_s, s_e_e, s_a_s, s_a_e, v_e, v_a):
    a = 1.0
    b = 1.0
    t1 = s_e_e / v_e
    t2 = s_a_e / v_a
    k1 = v_e / s_e_s
    k2 = v_a / s_a_s
    return a, b, t1, t2, k1, k2


def HeuristicTrans(s_e_s, s_e_e, s_a_s, s_a_e, v_e, v_a, h_a):
    a = 1.0
    b = 1.0
    t2 = s_a_e / v_a
    k2 = v_a / s_a_s
    if h_a < 0.0:
        if math.pow(v_e, 2) / (2.0 * h_a) < s_e_s:
            return a, b, 1e+8, t2, 1e+8, k2
        if math.pow(v_e, 2) / (2.0 * h_a) < s_e_e:
            ttc = CalEquation(0.5 * h_a, v_e, -s_e_s)
            k1 = 1.0 / ttc
            return a, b, 1e+8, t2, k1, k2
    ttc = CalEquation(0.5 * h_a, v_e, -s_e_s)
    t1 = CalEquation(0.5 * h_a, v_e, -s_e_e)
    k1 = 1.0 / ttc
    return a, b, t1, t2, k1, k2


def CalEquation(a, b, c):
    return (-b + math.sqrt(math.pow(b, 2) - 4.0 * a * c)) / (2.0 * a)


def f(x, y, a, c, t1, t2, k1, k2):
    return x * (1-x) * (-2 * k2 * y - y * a * c * t2 + 2 * a * t2 - y * a * t2)


def g(x, y, a, c, t1, t2, k1, k2):
    return y * (1-y) * (-2 * k1 * x - x * a * c * t1 + 2 * a * t1 - x * a * t1)


def calculateValue(initX, initY, dt, epoch, a, c, t1, t2, k1, k2):
    x = []
    y = []

    x.append(initX)
    y.append(initY)

    for index in range(epoch):
        tempx = x[-1] + (f(x[-1], y[-1], a, c, t1, t2, k1, k2)) * dt
        tempy = y[-1] + (g(x[-1], y[-1], a, c, t1, t2, k1, k2)) * dt

        x.append(tempx)
        y.append(tempy)
    return (x, y)


def PlotEvolution(prob_set, a, c, t1, t2, k1, k2):
    D = []
    for param in prob_set:
        x = param[0]
        y = param[1]
        d = calculateValue(x, y, 0.01, 1000, a, c, t1, t2, k1, k2)
        D.append(d)
    return D


if __name__ == '__main__':
    s_ego_s = 6.0
    s_ego_e = 7.0
    s_agent_s = 5.0
    s_agent_e = 5.5
    v_ego = 3.0
    v_agent = 2.0

    a, c, t1, t2, k1, k2 = GeneralTrans(s_ego_s, s_ego_e, s_agent_s, s_agent_e, v_ego, v_agent
                                        )

    prob_set = [[0.5, 0.5], [0.6, 0.3], [0.7, 0.1],
                [0.1, 0.3], [0.7, 0.9], [0.2, 0.8]]

    D = PlotEvolution(prob_set, a, c, t1, t2, k1, k2)
    plt.figure(1)
    plt.plot(D[0][0], D[0][1], label='x=0.5,y=0.5')
    plt.plot(D[1][0], D[1][1], label='x=0.6,y=0.3')
    plt.plot(D[2][0], D[2][1], label='x=0.7,y=0.1')
    plt.plot(D[3][0], D[3][1], label='x=0.1,y=0.3')
    plt.plot(D[4][0], D[4][1], label='x=0.7,y=0.9')
    plt.plot(D[5][0], D[5][1], label='x=0.2,y=0.8')

    plt.ylabel("$y$", fontsize=18)
    plt.xlabel("$x$", fontsize=18)
    plt.xticks([0, 0.2, 0.4, 0.6, 0.8, 1])
    plt.legend()

    prob_set = [[0.5, 0.5]]
    h_a_set = [0.5, 1.0, 1.5]
    D = []
    D_H = []
    for h_a in h_a_set:
        a, c, t1, t2, k1, k2 = HeuristicTrans(s_ego_s, s_ego_e, s_agent_s, s_agent_e, v_ego, v_agent, h_a
                                              )
        D = PlotEvolution(prob_set, a, c, t1, t2, k1, k2)
        D_H.append(D[0])
    # print(D_H)
    plt.figure(2)
    t_set = [x * 0.01 for x in range(500)]
    plt.plot(t_set, (D_H[0][0])[:500], label='heuristics a=0.5')
    plt.plot(t_set, (D_H[1][0])[:500], label='heuristics a=1.0')
    plt.plot(t_set, (D_H[2][0])[:500], label='heuristics a=1.5')
    # plt.plot(t_set, (D_H[3][0])[:500], label='heuristics a=-0.5')
    # plt.plot(t_set, (D_H[4][0])[:500], label='heuristics a=-1.0')
    # plt.plot(t_set, (D_H[5][0])[:500], label='heuristics a=-1.5')

    plt.ylabel("$x$", fontsize=18)
    plt.xlabel("$t$", fontsize=18)
    plt.legend()

    plt.figure(3)
    t_set = [x * 0.01 for x in range(500)]
    plt.plot(t_set, (D_H[0][1])[:500], label='heuristics a=0.5')
    plt.plot(t_set, (D_H[1][1])[:500], label='heuristics a=1.0')
    plt.plot(t_set, (D_H[2][1])[:500], label='heuristics a=1.5')
    # plt.plot(t_set, (D_H[3][1])[:500], label='heuristics a=-0.5')
    # plt.plot(t_set, (D_H[4][1])[:500], label='heuristics a=-1.0')
    # plt.plot(t_set, (D_H[5][1])[:500], label='heuristics a=-1.5')

    plt.ylabel("$y$", fontsize=18)
    plt.xlabel("$t$", fontsize=18)
    plt.legend()

    plt.show()
