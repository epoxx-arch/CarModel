# 车辆运动学模型 
车辆完全视为刚体，主要考虑车辆的位姿(位置坐标、航向角)、速度、前轮转角等的关系，不考虑任何力的影响。

**状态空间方程**：基于误差建立 

## 自行车模型 

做投影建立运动学关系  
\phi : 航向角 
\delta : 前轮转角
$$
    \dot{x} = v_x = v \cos(\phi) \\ 
    \dot{y} = v_y = v \sin(\phi) \\
    \dot{\phi} = \omega = \frac{v}{L} \tan(\delta) \\ 

$$
选取状态量$x = [x,y,\phi]$,控制量$u = [v,\delta]$,则对于参考轨迹上任一个参考点，r ,
$$
    \dot{x}_r = f(x_r,u_r) = [v \cos(\phi),v \sin(\phi),\frac{v}{L} \tan(\delta)]^T
$$

上式在参考点泰勒展开 
$$
    \dot{x} = f(x_r,u_r) + \frac{\partial f}{\partial x} (x_r - x) + \frac{\partial f}{\partial u} (u_r - u) + \cdots
$$

$$
    \dot{x} = f(x_r,u_r) + A(x_r - x) + B(u_r - u) + \cdots
$$

$$
A = [
    0, 0, -v \sin(\phi) \\
    0, 0, v \cos(\phi) \\
    0, 0, 0
]
$$

$$
B = [
    \cos(\phi), 0 \\
    \sin(\phi), 0 \\
    \frac{1}{L} \tan(\delta), \frac{v}{L} \frac{1}{\cos^2(\delta)}
]
$$

$$
\dot{x} - \dot{x_r} = A(x_r - x) + B(u_r - u) = A\tilde{x} + B\tilde{u}
$$ 

$$
\dot{\tilde{x}} = A\tilde{x} + B\tilde{u}
$$

$$
\dot{\tilde{x}} = (\tilde{x}(k+1) - \tilde{x}(k))/ T  = A\tilde{x} + B\tilde{u}
$$

$$
\tilde{x}(k+1) = \tilde{x}(k) + T(A\tilde{x}(k) + B\tilde{u}(k))
$$

$$
\tilde{x}(k+1) = (I + AT)\tilde{x}(k) + BT\tilde{u}(k)
$$ 

整合, 式中都是差分结果，与参考点的差值。
$$
\tilde{x}(k+1) = A \tilde{x}(k) + B \tilde{u}(k)
$$

