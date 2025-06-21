"""
This script reads a CSV file and creates subplots for the first two columns.
It sets the x-axis limits to the range of the index and the y-axis limits to the min and max values of each column.
It also adds grid lines to both subplots and adjusts the layout for better visibility.
"""

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df = pd.read_csv('file.csv')

# Create subplots
fig, axs = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

# Set desired x and y limits (customize these values)
x_min, x_max = 0, len(df) - 1
y1_min, y1_max = df.iloc[:, 0].min(), df.iloc[:, 0].max()
y2_min, y2_max = df.iloc[:, 1].min(), df.iloc[:, 1].max()

# Plot first column
axs[0].plot(df.index, df.iloc[:, 0])
axs[0].set_ylabel(df.columns[0])
axs[0].set_title('First Column')
axs[0].set_xlim(x_min, x_max)
axs[0].set_ylim(y1_min, y1_max)

# Plot second column
axs[1].plot(df.index, df.iloc[:, 1])
axs[1].set_ylabel(df.columns[1])
axs[1].set_title('Second Column')
axs[1].set_xlabel('Index')
axs[1].set_xlim(x_min, x_max)
axs[1].set_ylim(y2_min, y2_max)

# Add grid to both
for ax in axs:
    ax.grid(True)

# Adjust layout
plt.tight_layout()
plt.show()
