'''
1. Read several csv files, every row of csv file is a data pair that contains 3 value. Ignore the first row of every csv file. 
2. Merge all data pairs. 
3. Sort the merged data according to the third value in ascendent order. 
4. save data into a new csv file and use the first row ignored previously as the first row of the new file.
'''

def read_and_merge_csv_files(file_names):
    merged_data = []
    header = None

    # Read the specified CSV files
    for idx, file_name in enumerate(file_names):
        with open(file_name, 'r') as file:
            lines = file.readlines()
            if idx == 0:
                header = lines[0].strip()  # Store the first row as the header

            # Process the remaining rows, skipping the header
            for line in lines[1:]:
                # Split the row by space and strip any extra spaces or newlines
                row = line.strip().split()
                
                # Assume each row has exactly 3 values
                if len(row) == 3:
                    # Append the row as a tuple, converting the third value to float
                    merged_data.append((row[0], row[1], float(row[2])))

    return header, merged_data

def sort_data_by_third_value(data):
    # Sort the data by the third value (ascending order)
    return sorted(data, key=lambda x: x[2])

def save_to_csv_with_space(header, data, output_file):
    # Save the sorted data into a new file with space-separated values
    with open(output_file, 'w') as file:
        file.write(f"{header}\n")  # Write the original header from the first file
        for row in data:
            file.write(f"{row[0]} {row[1]} {row[2]}\n")

def main():
    # Specify the list of CSV files you want to merge
    file_names = ['file1.csv', 'file2.csv', 'file3.csv']  # Replace these with your actual file names
    output_file = "merged_sorted_data.csv"  # Output file

    # Read, merge, and sort data
    header, merged_data = read_and_merge_csv_files(file_names)
    sorted_data = sort_data_by_third_value(merged_data)

    # Save the sorted data to a new space-separated file with the original header
    save_to_csv_with_space(header, sorted_data, output_file)
    print(f"Data merged, sorted, and saved to {output_file}")

if __name__ == "__main__":
    main()
