
# Extracts the base name from a file name without its extension.

# ${variable%pattern}: Removes the smallest suffix matching pattern from the value of variable.
# ${variable%%pattern}: Removes the largest suffix matching pattern.
filename="data.txt"
basename="${filename%.*}"
echo "$basename"

# Extracting base name from a file path
full_path="/path/to/example.txt"
filename=$(basename "$full_path")
basename="${filename%.*}"
echo "$basename"

full_no_extension="${full_path%.*}"
echo "$full_no_extension"