'''
This script computes the optimal rotation matrix that aligns two sets of 3D normals using Singular Value Decomposition (SVD).
'''

import numpy as np

def fit_rotation(normals1, normals2):
    assert len(normals1) == len(normals2), "Mismatched number of normals"
    A = np.array(normals1).T  # shape: (3, N)
    B = np.array(normals2).T  # shape: (3, N)

    # Optional: normalize all vectors
    A = A / np.linalg.norm(A, axis=0)
    B = B / np.linalg.norm(B, axis=0)

    H = A @ B.T
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure right-handed coordinate system (det = 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    return R

def compute_residuals(normals1, normals2, R):
    normals1 = np.array(normals1)
    normals2 = np.array(normals2)

    # Normalize
    normals1 = normals1 / np.linalg.norm(normals1, axis=1, keepdims=True)
    normals2 = normals2 / np.linalg.norm(normals2, axis=1, keepdims=True)

    rotated_normals1 = (R @ normals1.T).T  # shape: (N, 3)

    # Euclidean residuals
    residuals = rotated_normals1 - normals2
    residual_norms = np.linalg.norm(residuals, axis=1)
    mean_residual_norm = np.mean(residual_norms)

    # Angular error
    dot_products = np.sum(rotated_normals1 * normals2, axis=1)
    dot_products = np.clip(dot_products, -1.0, 1.0)  # for numerical stability
    angles = np.arccos(dot_products)  # in radians
    mean_angle_deg = np.degrees(np.mean(angles))

    return mean_residual_norm, mean_angle_deg

normals1 = [
    [1, 0.5, 0.4],
    [0.999, 0.53, 0.36],
    [0.998, 0.48, 0.43]
]
normals2 = [
    [0.3, 0.4, 0.5],
    [0.31, 0.39, 0.51],
    [0.29, 0.38, 0.48]
]

R = fit_rotation(normals1, normals2)
print(R)


residual_norm, angle_error = compute_residuals(normals1, normals2, R)
print(f"Mean Euclidean residual norm: {residual_norm:.6f}")
print(f"Mean angular error (degrees): {angle_error:.6f}")