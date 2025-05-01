import pandas as pd
import matplotlib.pyplot as plt
import numpy as np  # cần import thêm numpy

file_path = '/home/huynn/huynn_ws/robot_catching_ws/rocat_ws/src/rocat_sim/results/NAE/cookie_box.xlsx'
object_name = 'cookie_box'

df = pd.read_excel(file_path, usecols=[0, 1, 2, 3], names=['X1', 'Y1', 'X2', 'Y2'])
df = df.dropna(subset=['X1', 'Y1', 'X2', 'Y2'])

plt.figure(figsize=(20, 12))
plt.plot(df['X1'], df['Y1'], linestyle='-', label='NAE')
plt.plot(df['X2'], df['Y2'], linestyle='-', label='LSTMencoder')

plt.xlabel('Time to Goal (frames)', fontsize=30)
plt.ylabel('IE', fontsize=30)
plt.title(f'Comparison of NAE, LSTMencoder Generalized model prediction result - {object_name}', fontsize=30)
plt.legend()
plt.grid(True)

# Set khoảng cách tick trục y là 0.05
min_y = min(df['Y1'].min(), df['Y2'].min())
max_y = max(df['Y1'].max(), df['Y2'].max())
plt.yticks(np.arange(np.floor(min_y*20)/20, np.ceil(max_y*20)/20 + 0.05, 0.05))

min_x = min(df['X1'].min(), df['X2'].min())
max_x = max(df['X1'].max(), df['X2'].max())
plt.xticks(np.arange(max_x, min_x - 1, -5))  # Giảm dần
# plt.xticks(np.arange(min_x, max_x + 1, 5))

plt.legend(fontsize=20)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)

plt.gca().invert_xaxis()

plt.tight_layout()
plt.show()
