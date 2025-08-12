'''
1. conversion between rotation matrix and euler angle
2. angle addition
'''

import numpy as np
from scipy.spatial.transform import Rotation as R

def euler_to_rotation_matrix(euler_angles, order='zyx'):
    """
    Convert Euler angles to rotation matrix.
    
    Parameters:
        euler_angles: list or np.ndarray of shape (3,) - angles in radians.
        order: str - rotation order, e.g., 'xyz', 'zyx', etc.
    
    Returns:
        np.ndarray of shape (3, 3) - rotation matrix.
    """
    r = R.from_euler(order, euler_angles)
    return r.as_matrix()

def rotation_matrix_to_euler(R_mat, order='xyz'):
    """
    Convert rotation matrix to Euler angles.
    
    Parameters:
        R_mat: np.ndarray of shape (3, 3) - rotation matrix.
        order: str - rotation order, e.g., 'xyz', 'zyx', etc.
    
    Returns:
        np.ndarray of shape (3,) - Euler angles in radians.
    """
    r = R.from_matrix(R_mat)
    return r.as_euler(order)

def add_euler_angles(euler1, euler2, order='zyx'):
    """
    Add two Euler angles (i.e., compose their rotations).
    
    Parameters:
        euler1, euler2: list or np.ndarray of shape (3,) - angles in radians.
        order: str - rotation order (e.g., 'xyz').
    
    Returns:
        np.ndarray of shape (3,) - Combined Euler angles in radians.
    """
    r1 = R.from_euler(order, euler1)
    r2 = R.from_euler(order, euler2)
    r_combined = r1 * r2  # Equivalent to matrix multiplication: R1 @ R2
    return r_combined.as_euler(order)

# Example usage:
if __name__ == "__main__":
    # Euler angles in radians
    euler1 = [-0.15, -0.1, 0]
    euler2 = [0.15, 0.1, 0]
    order = 'zyx'

    # Convert to rotation matrix
    R_mat = euler_to_rotation_matrix(euler1, order)
    print("Rotation matrix:\n", R_mat)

    # Convert back to Euler angles
    recovered_angles = rotation_matrix_to_euler(R_mat, order)
    print("Recovered Euler angles (radians):", recovered_angles)
    

    order = 'zyx'

    result = add_euler_angles(euler1, euler2, order)
    print("Combined Euler angles (radians):", result)

    # First rotation: 30 deg about Z
    q1 = R.from_euler('z', 30, degrees=True)

    # Second rotation: 45 deg about Y
    q2 = R.from_euler('y', 45, degrees=True)

    # Combined rotation
    q_total = q2 * q1  # apply q1 first, then q2

    print("Combined rotation matrix:\n", q_total.as_matrix())
    print("Combined quaternion:\n", q_total.as_quat())  # [x, y, z, w]