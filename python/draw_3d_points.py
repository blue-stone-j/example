'''
1. load file
2. use first column to determine color
3. use 2~4 columns as x y z of points
4. plot points and connect points with black lines
'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_file(filename):
    with open(filename, 'r') as file:
        # lines = file.readlines()
        lines = [line.strip() for line in file if line.strip()]

    colors=[]
    for line in lines:
        colors.append(int(line.strip().split()[0])/)

    # Parse points
    points = np.array([list(map(float, line.strip().split()))[1:4] for line in lines])

    return points, colors

def plot_line_and_points( points, colors):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    sc= ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, cmap='viridis')
    ax.plot(points[:, 0], points[:, 1], points[:, 2],color='black',lineWidth=1)

    # Labels and legend
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    plt.colorbar(sc, ax=ax)
    plt.show()


filename = 'file.txt' # use whitespace as separator

points, colors = read_file(filename)
plot_line_and_points(points, colors)
