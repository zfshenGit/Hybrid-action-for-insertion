#导入所需的包
import numpy as np  
from matplotlib import pyplot as plt

#导入npy文件路径位置
test = np.load('./returns.npy')
print(len(test))
x = list(range(1, 300))
np.savetxt('returns.txt', test)  

plt.plot(x,test[0:299])
plt.show()

