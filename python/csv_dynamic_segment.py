'''
1. read a csv file. 
2. divide data into segments according the third value of every row by range 0.2, for example, the rows whose third 
   value is between 4.0 and 4.2 will be put together. 
3. calculate average of the first and second  value of rows in same segments. 
4. make a row from a segment, use averages as the first and second value, use the middle value as the third value, 
   such as 4.1(because of 4.0 and 4.2 mentioned above) .  
5. write new rows into new csv file. What we know: the original data have been sorted according to the third value of 
   every row.

unfilled csv whose index doesn't start from 0
'''

import csv
import numpy as np

# Function to divide data into segments and calculate averages
def process_csv(input_file, output_file, segment_size=0.2, decimal_places=3):
    segments = []
    current_segment = []
    start_value = None
    
    # Read input CSV with space as the delimiter
    with open(input_file, 'r') as infile:
        reader = csv.reader(infile, delimiter=' ')
        for row in reader:
            row = list(map(float, row))  # Convert strings to floats
            third_value = row[2]
            
            if start_value is None:
                start_value = third_value
            
            if third_value >= start_value and third_value < start_value + segment_size:
                current_segment.append(row)
            else:
                if current_segment:
                    segments.append(current_segment)
                start_value = third_value
                current_segment = [row]
        
        # Add the last segment
        if current_segment:
            segments.append(current_segment)
    
    # Process each segment
    processed_data = []
    for segment in segments:
        if len(segment) > 0:
            # Calculate average of first and second values
            avg_first = np.mean([row[0] for row in segment])
            avg_second = np.mean([row[1] for row in segment])
            # Determine the middle value for the third column
            mid_value = (segment[0][2] + segment[-1][2]) / 2.0
            # Format the values to the specified decimal places
            formatted_row = [f"{avg_first:.{decimal_places}f}", f"{avg_second:.{decimal_places}f}", f"{mid_value:.{decimal_places}f}"]
            processed_data.append(formatted_row)
    
    # Write output CSV with space as the delimiter
    with open(output_file, 'w', newline='') as outfile:
        writer = csv.writer(outfile, delimiter=' ')
        writer.writerows(processed_data)

# Example usage:
input_file = 'input.csv'
output_file = 'output.csv'
process_csv(input_file, output_file, decimal_places=3)  # Control decimal precision here
