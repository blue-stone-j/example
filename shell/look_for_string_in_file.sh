# This command checks if the string "realtime" is present in the file params.yaml.
[ "`cat "params.yaml" | grep realtime`" ] && echo "0" || echo "1"

# A more efficient and idiomatic way to check for the presence of a string in a file
grep -q realtime params.yaml && echo "0" || echo "1"
