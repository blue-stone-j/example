'''
1. draw 3D poses with direction vectors
2. the begin of the vector is position, direction is normal direction of quaternion.
'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # <-- Required for '3d' projection
from scipy.spatial.transform import Rotation as R

def set_axes_equal(ax):
    """Set 3D plot axes to equal scale."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    max_range = max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - max_range/2, x_middle + max_range/2])
    ax.set_ylim3d([y_middle - max_range/2, y_middle + max_range/2])
    ax.set_zlim3d([z_middle - max_range/2, z_middle + max_range/2])

def quaternion_to_direction(q):
    rot = R.from_quat(q)
    return rot.apply([0, 0, 1])  # Z-axis direction

def draw_pose(ax, position, quaternion, color='blue'):
    direction = quaternion_to_direction(quaternion)
    ax.quiver(*position, *direction, length=0.5, normalize=True, color=color)

def plot_pose_sets(pose_sets):
    num_sets = len(pose_sets)
    fig = plt.figure(figsize=(5 * num_sets, 5))
    
    for i, poses in enumerate(pose_sets):
        ax = fig.add_subplot(1, num_sets, i + 1, projection='3d')
        ax.set_title(f'Set {i + 1}')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        for pos, quat in poses:
            draw_pose(ax, pos, quat)
        set_axes_equal(ax)
        # ax.set_box_aspect([1, 1, 1])
        ax.grid(True)
    
    plt.tight_layout()
    plt.show()

# Example usage
pose_sets = [
    [([0, 0, 0], [0, 0, 0, 1]), ([1, 0, 0], [0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])],
    [([0, 1, 0], [0, np.sin(np.pi/4), 0, np.cos(np.pi/4)]), ([1, 1, 0], [np.sin(np.pi/4), 0, 0, np.cos(np.pi/4)])]
]

plot_pose_sets(pose_sets)
