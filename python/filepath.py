'''
manipulate file paths
'''

from pathlib import Path
import os

def split_path(path):
    directory = os.path.dirname(path)
    filename_with_ext = os.path.basename(path)
    filename, ext = os.path.splitext(filename_with_ext)
    return directory, filename, ext

def combine_path(directory, filename, extension):
    return os.path.join(directory, filename + extension)

def combine_folder_and_filename(directory, filename_no_ext):
    return os.path.join(directory, filename_no_ext)

# Example usage
filepath = "/home/user/data/image001.png"
directory, filename, extension = split_path(filepath)

print("Directory:", directory)
print("Filename:", filename)
print("Extension:", extension)
print()

# Combining paths
combined_path = combine_path(directory, filename, extension)
print("Combined Path:", combined_path)
print()

# Combining directory and filename without extension
combined_dir_filename = combine_folder_and_filename(directory, filename)
print("Combined Directory and Filename:", combined_dir_filename)


### traverse directories
root_path = Path("/path/to/your/directory")
# Traverse all directories and subdirectories
for path in root_path.rglob("*"):
    if path.is_dir():
        print(path)
# only direct subdirectories of the root
for path in root_path.iterdir():
    if path.is_dir():
        print(path)

# dirnames: list of subdirectories in dirpath
# filenames: list of non-directory files in dirpath
for dirpath, dirnames, filenames in os.walk(root_path):
    print("Current folder:", dirpath)
    print("Subfolders:", dirnames)
    print("Files:", filenames)

# Create a directory if it does not exist
if not os.path.exists(directory):
    os.makedirs(directory)

# Get the folder name from a file path
folder = os.path.dirname(filepath)
print(folder)

from pathlib import Path
# Get the folder name using pathlib
folder = filepath.parent