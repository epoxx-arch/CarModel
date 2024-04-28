import math
class IDM:
    def __init__(self, v0, T, a, b, s0, delta):
        self.v0 = v0 # desired velocity
        self.T = T # safe time headway
        self.a = a # maximum acceleration
        self.b = b # comfortable deceleration
        self.s0 = s0 # minimum gap 最小安全距离
        self.delta = delta # acceleration exponent 加速度指数，

    def __call__(self, v, s):
        print("v,s",v,s)
        s_star = self.s0 + max(0, v * self.T + v * (v - self.v0) / (2 * np.sqrt(self.a * self.b)))
        print("前车期望距离:",s_star)
        return  self.a * (1 - (v / self.v0) ** self.delta - (s_star / s) ** 2) ** 1

    def cal(self, v, s):
        _v = [v]
        _a = [0]
        _s = [s]
        for i in range(200):
            if i==0:continue
            _a.append(self(_v[-1], _s[-1]))
            _v.append(_v[-1] + _a[-1] * 0.1)
            _s.append(_s[-1] + _v[-1] * 0.1)
        return _v, _a, _s
    
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np
    v0 = 33 # desired velocity
    T = 1.5 # 车头时距
    a = 6 # 最大加速度 
    b = 1.5 # 最小减速度 
    s0 = 2.0 # 最小安全间隔 
    delta = 2.0 # 系数 
    # model = IDM(v0, T, a, b, s0, delta)
    # v, a, s = model.cal(8, 100) # v，s
    #  跟车距离很大时，
    plt.figure()
    for coeff in range(1,6):
        model = IDM(v0, T, a, b, s0, coeff)
        _v, _a, _s = model.cal(8, 10)
        # plt.plot(_v, label=str(coeff) + '_v')
        plt.plot(_a,label=str(coeff) + '_a')
    # plt.plot(v, label='v')
    # plt.plot(a, label='a')
    # plt.plot(s, label='s')
    plt.legend()
    plt.show()