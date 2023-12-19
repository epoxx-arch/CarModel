# Interduction

## 1.1 PDE 
field + derivatives

## 1.2 CODE 


## 1.3 Coupled iterated maps 

## 1.4 Cellular automata 

## 1.5 Discrete state variables, continuous time

## 1.6 .Static models 



# Model Criteria 
跟车模型的基本假设：

- 加速度是速度的严格递增函数,如果没有其它限制，车辆将增加值期望速度 $v_0$
$$  \frac{\partial{a_{mic}(s,v,v_l)}}{\partial{v}}<0, \lim_{s \to \inf}{a_{mic}(s, v_0, v_l)} = 0 $$
- 加速度是与前车距离的递增函数。
  $$ \frac{\partial{a_{mic}(s,v,v_l)}}{\partial{s}} >= 0, \lim_{s \to \inf} \frac{\partial{a_{mic}(s,v,v_l)}}{\partial{s}} = 0 $$

如果其它车辆或障碍物在交互范围之外，因此不影响驾驶行为，那么不等式将变为等式。(没有它车)
$$ a_{free}(v) = \lim_{s \to \inf} \frac{\partial{a_{mic}(s,v,v_l)}}{\partial{s}} >= a_{mic}(s,v,v_l) $$

- 加速度是随着前车速度的增加而增加的函数，加速度随着接近前车的速度而减小。
$$ \frac{\partial{a_{mic}(s,v,v_l)}}{\partial{\Delta{v}}} <= 0, \frac{\partial{a_{mic}(s,v,v_l)}}{\partial{v_l}} >=0 $$

- 保持与前车的最小间隙 $s_0$，如果由于过去事件导致间隙小于 $s_0$，则车辆不会倒退。
  $$ a_{mic}(s,v,v_l) = 0, for\space all\space v_l >= 0, s<=s_0 $$


*这些对加速度函数的要求自然会导致用耦合映射表示的模型的速度函数 的条件。  一个满足这些要求的车辆跟随模型在描述单车道交通中可能出现的所有情况方面是完整的*
可得出结论：
    - 所有车辆间的交互作用都具有有限的范围
    - 跟随车辆不会被“拖拽”
    - 存在一个平衡速度，它具有已经假设的最优速度函数的属性: --> 具有唯一的稳态流量关系。
  $$ v_e^{\prime}(s) >= 0, v_e{(0)} = 0, \lim_{s \to \inf}{(s)} = v_0$$

## 1.2 Gipps Model 
最简单的完整且无事故模型，但存在不现实的加速度曲线
*假设*：
- 制动操作总是以恒定的减速度b执行，舒适减速度和最大减速度之间没有区别。-->前车制动距离： $\Delta x_l = \frac{v_l^2}{2b}$
- 不变的反应时间。--> 停车距离：$\Delta x = v \Delta t +\frac{v^2}{2b}$
- 即使前方的车辆突然减速停车，与前车之间的距离差也不应该小于最小距离 $s_0$
  $$s >= s_0 + v \Delta t +\frac{v^2}{2b} - \frac{v_l^2}{2b} $$
- 安全速度 
  $$v_{safe} = -b \Delta t + \sqrt{b^2 \Delta{t^2} + v_l^2 + 2b(s-s_0)}$$ 
  
简化Gipps模型：
$$v(t+\Delta{t}) = min[v+a\Delta{t}, v_0, v_{safe}(s,v_l)] $$
- 模拟的更新时间步长等于反应时间，
- 如果当前速度大于 $v_{safe} - a\Delta{t} \space or \space v_0 - a\Delta{t}$,车辆将在下一个时间步骤中达到$v_0$和$v_{safe}$的最小值之一。否则，车辆将以恒定的减速度a加速，直到达到安全速度或所需速度。

### 1.2.1 Model Recurrent 


```python
class GippsModel():
  def __init__(self, v0, a, b, s0, delta_t):
    self.v0 = v0 # 期望速度 v0
    self.a = a # 恒定加速度 a
    self.b = b # 恒定减速度 b 
    self.s0 = s0 # 最小安全距离 s0
    self.delta_t = delta_t
  
  # v: 当前速度, s: 当前距离, vl: 前车速度
  def __call__(self, v, s, vl):
    delta_x = vl * self.delta_t + vl**2 / (2 * self.b) # 
    v_safe = self.v0 + self.a * self.delta_t # 
    # print(self.s0 + delta_x)
    if s < self.s0 + delta_x:
      return 0
    elif v < v_safe:
      return min(v + self.a * self.delta_t, v_safe)
    else:
      return min(v + self.a * self.delta_t, self.v0)

  def __repr__(self):
    return "Gipps Model" 
  
  def cal(self, v, s, vl):
    _v = [v]
    _s = [s]
    _a = [0]
    t = [i / 10 for i in range(80)]
    for i in t:
        if i == 0:
            continue
        _v.append(self.__call__(_v[-1], _s[-1], vl))
        _a.append((_v[-1] - _v[-2]) / 0.1 ) 
        _s.append(_s[-1] - (_v[-1] - vl) * self.delta_t)
        # print(_v[-1], _s[-1])
    return t, _v, _s, _a
```

## 1.3 IDM 

- 满足基本假设
- 与前车保持平衡的保险杠到保险杠不小于“安全距离” $s_0+vt$， $t$ 是与前车的时间间隙，$s_0$ 是最小间隙。
- 智能控制如何接近较慢的车辆。
- 不同驾驶模式之间的转换是平滑的，Jerk在所有时刻都是有限的，
- 每个模型参数应仅描述驾驶行为的一个方面。


$$\dot{v} = a[1-(\frac{v}{v_0})^\sigma - (\frac{s^\star(v,\Delta{v})}{s})^2]$$

- $s^\star(v,\Delta{v}) = s_0 + \max{(0,vT + \frac{v \Delta v}{2 \sqrt{ab}})}$ 是期望的安全距离，它是与前车的距离，前车速度和期望速度的函数。

- 第一项是期望加速度，第二项是速度的负反馈，第三项是与前车的距离的负反馈。第二项用来促使车辆达到期望速度，第三项用来保持与前车的安全距离。
- 第二项：促使自车加速，直到达到期望速度。自车速度较小，第二项较小，自车加速度较大；
- 第三项：保持与前车的安全距离。与前车的距离越小，第三项越大，自车加速度越小。

## 1.3.1 Model Recurrent

```python
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
        return  self.a * (1 - (v / self.v0) ** self.delta - (s_star / s) ** 2)

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
```
## 1.3.2 IDM Expand 


