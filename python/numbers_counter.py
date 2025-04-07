"""
This script reads integers from a CSV file, counts the frequency of each integer,
and plots a bar chart of the frequencies.
It uses the `csv` module for reading the CSV file and `matplotlib` for plotting.
"""

import csv
from collections import Counter
import matplotlib.pyplot as plt

# Path to CSV
csv_file = 'data.csv'

values = []

# Read integers from CSV
with open(csv_file, newline='') as f:
    reader = csv.reader(f)
    for row in reader:
        if row:
            try:
                values.append(int(row[0]))
            except ValueError:
                continue  # skip non-integer rows

# Count frequencies
counter = Counter(values)
x = sorted(counter.keys())
y = [counter[val] for val in x]

# Plot
plt.figure(figsize=(8, 5))
plt.bar(x, y)
plt.xlabel('Value (x)')
plt.ylabel('Count (y)')
plt.title('Frequency of Each Value in CSV')
plt.grid(True, axis='y')
plt.tight_layout()
plt.show()
