
file="./info.txt"

# Check if the file exists, if not create it. if exists, automatically adds a newline
echo "information1" >> "$file"

# \n won't be interpreted as a newline in this echo command
echo "information2\n" >> "$file"

# enables interpretation of backslash escapes
echo -e "information2\n" >> "$file"

# strict control of newlines
name="John Doe"
datetime=$(date +"%Y-%m-%d %H:%M:%S")
printf "%s %s\n" "$name" "$datetime" >> "$file"
printf "%s\n%s\n" "$name" "$datetime" >> "$file"

# overwriting the file with new content
echo "information3" > "$file"