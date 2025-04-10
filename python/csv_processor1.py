import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

'''
function:
1. read a csv file(take as A) and take first column as index, which starts from 0 and continuous, take second column as value. 
2. then read another csv file(take as B). use third column as x. 
3. use int(5*x) as index to find corresponding value from file A. 
4. use first column add this value as y. At last, draw x and y.
'''

# Read CSV file A with space as the delimiter and set the first column as index
file_A = 'file_A.csv'
df_A = pd.read_csv(file_A, delim_whitespace=True, index_col=0)

# Read CSV file B with space as the delimiter and skip the first row
file_B = 'file_B.csv'
df_B = pd.read_csv(file_B, delim_whitespace=True, skiprows=1)

# Ensure that the third column is used as x and apply int(5*x) to compute the index
x_values = df_B.iloc[:, 2].to_numpy()  # Convert to NumPy array

y_values = []

# Iterate over the x_values to find the corresponding y using file A
for idx, x in enumerate(x_values):
    index = int(5 * x)
    if index in df_A.index:
        # Get the corresponding value from file A (second column in file A)
        y = df_A.iloc[index, 0] + df_B.iloc[idx, 0]  # Add first column of file B as y
        y_values.append(y)
    else:
        # Handle case where index is not in file A (optional, depending on your data)
        y_values.append(None)

# Convert y_values list to a NumPy array
y_values = np.array(y_values)

# Plot the data (x vs y)
plt.plot(x_values, y_values, marker='o')
plt.xlabel('X values from file B (3rd column)')
plt.ylabel('Y values (file A corresponding value + file B 1st column)')
plt.title('X vs Y Plot')
plt.grid(True)
plt.show()
