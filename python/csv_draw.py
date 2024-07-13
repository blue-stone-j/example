'''
1. For single csv file, read first and third column as y and x respectively, except first row. Draw this line on a chart. 
2. Draw a line like this from every csv file.
3. Specify csv files manually,
'''
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Function to read CSV and plot data with custom axis limits
def plot_csv_files(csv_files, x_range=None, y_range=None):
    plt.figure(figsize=(10, 6))

    for csv_file in csv_files:
        try:
            # Read the CSV file using space as the delimiter, skipping the first row (assuming it contains headers)
            df = pd.read_csv(csv_file, delim_whitespace=True, skiprows=1)

            # Check if the file has at least 3 columns
            if df.shape[1] >= 3:
                # Get first and third columns as y and x respectively and convert to numpy arrays
                y = pd.to_numeric(df.iloc[:, 0].to_numpy(), errors='coerce')  # First column (Y-axis)
                x = pd.to_numeric(df.iloc[:, 2].to_numpy(), errors='coerce')  # Third column (X-axis)

                # Apply square root to the x values
                x = np.sqrt(x)

                # Drop rows with NaN values (in case of non-numeric or missing data)
                valid_mask = ~np.isnan(x) & ~np.isnan(y)
                x = x[valid_mask]
                y = y[valid_mask]

                # Plot the line for this file if there's valid data
                if len(x) > 0 and len(y) > 0:
                    plt.plot(x, y, label=f'File: {csv_file}')
                else:
                    print(f"No valid data to plot in {csv_file}.")
            else:
                print(f"File {csv_file} doesn't have enough columns to plot.")

        except Exception as e:
            print(f"Error processing file {csv_file}: {e}")

    # Set axis labels and title
    plt.xlabel('Square Root of X (Third Column)')
    plt.ylabel('Y (First Column)')
    plt.title('Line Plot from Multiple CSV Files with Square Root of X')

    # Set custom axis ranges if provided
    if x_range:
        plt.xlim(x_range)
    if y_range:
        plt.ylim(y_range)

    plt.legend()
    plt.grid(True)
    plt.show()

# Replace with the actual directory path where the CSV files are stored
# csv_files = glob.glob("path_to_your_directory/*.csv")

# Manually specify the list of CSV file paths
csv_files = [
    "path_to_your_directory/file1.csv",
    "path_to_your_directory/file2.csv",
    "path_to_your_directory/file3.csv"
]

# Example: Specify the range for x and y axes
x_range = (0, 10)  # Set the range for the X-axis (adjusted for square root)
y_range = (0, 50)  # Set the range for the Y-axis

# Call the function to plot the specified files with custom axis ranges
plot_csv_files(csv_files, x_range=x_range, y_range=y_range)
