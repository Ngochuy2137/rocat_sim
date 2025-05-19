import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

object_name = 'boomerang'
file_path = f'/home/server-huynn/workspace/robot_catching_project/experiment/nae_static/utils/results/1-catching-sim-exp/{object_name}/synthesis.ods'

# 1) Đọc file
df = pd.read_excel(
    file_path,
    usecols=[0,1,2,3],
    names=['X1','Y1','X2','Y2']
)

# 2) Tách hai DataFrame độc lập, drop riêng
df1 = df[['X1','Y1']].dropna().astype(float)
df2 = df[['X2','Y2']].dropna().astype(float)

# 3) (Tùy) Sort để line không zig-zag
df1 = df1.sort_values('X1', ascending=False)
df2 = df2.sort_values('X2', ascending=False)

# 4) Vẽ
plt.figure(figsize=(20,12))
plt.plot(df1['X1'], df1['Y1'], label='NAE')
plt.plot(df2['X2'], df2['Y2'], label='LSTMencoder')

plt.xlabel('Time to Goal (frames)', fontsize=24)
plt.ylabel('IE (m)', fontsize=24)
plt.title(f'Online prediction – {object_name}', fontsize=26)
plt.grid(True)
plt.legend(fontsize=20)

# 5) Giới hạn trục X xuống đúng min/max của cả hai
max_x1, min_x1 = df1['X1'].max(), df1['X1'].min()
max_x2, min_x2 = df2['X2'].max(), df2['X2'].min()
overall_max = max(max_x1, max_x2)
overall_min = min(min_x1, min_x2)
plt.xlim(overall_max, overall_min)
plt.xticks(
    np.arange(overall_max, overall_min-1, -5),
    fontsize=18
)

# 6) Thiết lập yticks với bước 0.1
min_y = min(df1['Y1'].min(), df2['Y2'].min())
max_y = max(df1['Y1'].max(), df2['Y2'].max())
yticks = np.arange(
    np.floor(min_y*10)/10,      # làm tròn xuống 1 chữ số
    np.ceil(max_y*10)/10 + 0.1, # làm tròn lên và cộng thêm 1 bước
    0.1                         # bước nhảy 0.1
)
plt.yticks(yticks, fontsize=18)

plt.tight_layout()
plt.show()
