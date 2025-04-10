"""
# fit_1d.py
This script demonstrates how to fit a 1D polynomial to a set of data points using NumPy and Matplotlib.
It generates a 9th-order polynomial fit and visualizes the results.
"""

import numpy as np
import matplotlib.pyplot as plt

# Example data
x = np.array([1, 2, 3, 4, 5])
y = np.array([1, 4, 9, 16, 25])

# Fit a 9th-order polynomial, ordered from highest to lowest power.
coeffs = np.polyfit(x, y, 9)

# Create the polynomial function. 
poly_func = np.poly1d(coeffs)

# Evaluate the polynomial
x_fit = np.linspace(min(x), max(x), 100)
y_fit = poly_func(x_fit)

# Plot
plt.scatter(x, y, label='Data')
plt.plot(x_fit, y_fit, label='9th-order fit', color='r')
plt.legend()
plt.show()
