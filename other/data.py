#导入所需的包
import numpy as np  
from matplotlib import pyplot as plt

#导入npy文件路径位置
test = np.load('./results/peginhole/proposed_square_peg1/terminals.npy')
# x = 0:1:len(test)
# print(test[0:20])
np.savetxt('terminals.txt', test)  

# plt.plot(np.array(test),'ro')
# plt.show()

