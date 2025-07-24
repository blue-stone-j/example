'''
Plotting multiple data series from text files in a directory.
This script reads all text files in a specified directory, extracts each column as a separate data series,
and plots them on the same graph.
'''

import matplotlib.pyplot as plt
import numpy as np
import glob

file_pattern = 'data/*.txt'
file_list = glob.glob(file_pattern)

all_series = []

for file in file_list:
    data = np.loadtxt(file, delimiter=',')
    if data.ndim == 1:
        data = data.reshape(1, -1)
    for col_idx in range(data.shape[1]):
        all_series.append(data[:, col_idx])

plt.figure()
for idx, series in enumerate(all_series):
    plt.plot(series, label=f'File Column {idx+1}')
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Each Column from All Files as a Data Series')
plt.legend()
plt.grid(True)
plt.show()
