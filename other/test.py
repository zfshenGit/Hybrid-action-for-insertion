from matplotlib import pyplot as plt
import numpy as np
x=np.linspace(-3,3,100)#X的范围位于【-3，3】
y=np.sin(np.pi*x)
plt.title(r'$y=\sin(\pi\times x)$')#这是latex的表达式，与matlplotlib兼容
plt.plot(x,y,'ro')
#plt.plot(x, y, ls='-', lw=2, label='xxx', color='g' ) ls是线条的风格，lw是线条的宽度，label为标签文本
plt.show()#展示图象

