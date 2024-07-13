'''
1. read a csv file. 
2. divide data into segments according the third value of every row by range 0.2, for example, the rows whose third 
   value is between 4.0 and 4.2 will be put together. 
3. calculate average of the first and second  value of rows in same segments. 
4. make a row from a segment, use averages as the first and second value, use the middle value as the third value, 
   such as 4.1(because of 4.0 and 4.2 mentioned above) .  
5. write new rows into new csv file. 

What we know: the original data have been sorted according to the third value of every row.

filled csv whose index starts from 0
'''

import csv
import numpy as np

# Function to divide data into fixed segments (0.0-0.2 as 1, 0.2-0.4 as 2, etc.) and calculate averages
def process_csv(input_file, output_file, segment_size=0.2, decimal_places=3):
    segments = {}
    
    # Read input CSV with space as the delimiter
    with open(input_file, 'r') as infile:
        reader = csv.reader(infile, delimiter=' ')
        for row in reader:
            row = list(map(float, row))  # Convert strings to floats
            third_value = row[2]
            
            # Find the appropriate segment integer (e.g., 0.0-0.2 -> 1, 0.2-0.4 -> 2)
            segment_number = int(third_value // segment_size) + 1
            
            if segment_number not in segments:
                segments[segment_number] = []
            segments[segment_number].append(row)
    
    # Process each segment
    processed_data = []
    for segment_number, segment in sorted(segments.items()):
        if len(segment) > 0:
            # Calculate average of first and second values
            avg_first = np.mean([row[0] for row in segment])
            avg_second = np.mean([row[1] for row in segment])
            # Rearrange columns: third column (segment number) becomes the first, averages as second and third
            formatted_row = [segment_number, round(avg_first, decimal_places), round(avg_second, decimal_places)]
            processed_data.append(formatted_row)
    
    # Find the maximum index to ensure continuity from 0 to max index
    max_index = processed_data[-1][0] if processed_data else 0
    
    # Ensure index starts from 0 and is continuous
    filled_data = []
    for i in range(max_index + 1):
        # Find the row with the current index, or insert [i, 0, 0] if missing
        existing_row = next((row for row in processed_data if row[0] == i), None)
        if existing_row:
            filled_data.append(existing_row)
        else:
            filled_data.append([i, 0, 0])  # Fill missing rows with zeros

    # Write output CSV with space as the delimiter
    with open(output_file, 'w', newline='') as outfile:
        writer = csv.writer(outfile, delimiter=' ')
        writer.writerows(filled_data)

# Example usage:
input_file = 'input.csv'
output_file = 'output.csv'
process_csv(input_file, output_file, decimal_places=3)  # Control decimal precision here
