
############################################################ 
# create_potential_map.cppで作成したcsvファイルを3Dグラフ化する
############################################################

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys

# ファイルのパスを取得
file_name = sys.argv[1]

# パラメータを取得
param = pd.read_csv(file_name, header=None, usecols=[1])
range_x = param[1][0]
range_y = param[1][1]
unit = param[1][2]

# 1行目X_RANGE, 2行目Y_RANGE, 3行目UNIT, 4行目空白をスキップ
data = pd.read_csv(file_name, header=None, skiprows=4)

# インスタンス作成
fig = plt.figure()
ax = fig.gca(projection='3d')

# gridのサイズ調整
Xgrid = (data.columns * unit) - range_x
Ygrid = (data.index * unit) - range_y

X, Y = np.meshgrid(Xgrid, Ygrid)
Z = data.values

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('potential')

ax.plot_surface(X, Y, Z, cmap='jet') # 形と色を指定しながらプロット
plt.show()
