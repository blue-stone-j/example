'''
manipulate file paths
'''

import os

def split_path(path):
    directory = os.path.dirname(path)
    filename_with_ext = os.path.basename(path)
    filename, ext = os.path.splitext(filename_with_ext)
    return directory, filename, ext

def merge_path(directory, filename, extension):
    return os.path.join(directory, filename + extension)

def merge_dir_and_filename(directory, filename_no_ext):
    return os.path.join(directory, filename_no_ext)

# Example usage
path = "/home/user/data/image001.png"
directory, filename, extension = split_path(path)

print("Directory:", directory)
print("Filename:", filename)
print("Extension:", extension)
print()

# Merging paths
merged_path = merge_path(directory, filename, extension)
print("Merged Path:", merged_path)
print()

# Merging directory and filename without extension
merged_dir_filename = merge_dir_and_filename(directory, filename)
print("Merged Directory and Filename:", merged_dir_filename)