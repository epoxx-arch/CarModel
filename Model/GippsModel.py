import matplotlib.pyplot as plt
import math 
class GippsModel():
  def __init__(self, v0, a, b, s0, delta_t):
    self.v0 = v0 # 期望速度 v0
    self.a = a # 恒定加速度 a
    self.b = b # 恒定减速度 b 
    self.s0 = s0 # 最小安全距离 s0
    self.delta_t = delta_t
  
  # v: 当前速度, s: 当前距离, vl: 前车速度
  def __call__(self, v, s, vl):
    delta_x = vl * self.delta_t + vl**2 / (2 * self.b) # 停车距离
    v_safe =-self.b * self.delta_t + math.sqrt(self.b**2 * self.delta_t**2 + vl **2 + 2 * self.b *(s - delta_x)) # 以恒定加速度得到的安全下一帧的速度
    print("safe_v:", v_safe)
    return min(min(v + self.a * self.delta_t, self.v0),v_safe)

  def __repr__(self):
    return "Gipps Model" 
  
  def cal(self, v, s, vl):
    _v = [v]
    _s = [s]
    _a = [0]
    t = [i / 10 for i in range(0,80)]
    for i in t:
        if i == 0:
            continue
        _v.append(self.__call__(_v[-1], _s[-1], vl))
        _a.append((_v[-1] - _v[-2]) / self.delta_t ) 
        _s.append(_s[-1] - (_v[-1] - vl) * self.delta_t)
        # print(_v[-1], _s[-1])
    return t, _v, _s, _a


if __name__ == "__main__":
  model = GippsModel(10, 2, 3, 3, 0.1) # 
  t, v, s, a = model.cal(6, 8, 8)

  print(t,v,s)
  plt.plot(t, v)
  plt.plot(t,a)
#   plt.show()
  plt.plot(t, s)
  plt.plot(t, [8 for i in range(len(t))])
  plt.legend(['v','a', 's','l_v'])
  plt.show()