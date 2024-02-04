import matplotlib.pyplot as plt 
import numpy as np 


x = np.linspace(0, 10, 100)

y = x * np.tan(np.pi * 10/ 180)

# 调整坐标轴坐标均匀分布 
plt.plot(x, y)
plt.xticks(np.arange(0, 10, 1))
plt.yticks(np.arange(0, 10, 1))
plt.show()