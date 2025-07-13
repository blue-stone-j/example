list="a b c d" # defines the list as a space-separated string.
for item in $list; do
    echo "$item"
done
echo ""

for item in $list
do
    echo "$item"
done
echo ""

for i in 1 2 3; do echo $i; done
echo ""

list=("a" "b" "c d" "e") # defines the list as an array with elements that can contain spaces.

# safely expands to each element, preserving spaces within elements.
for item in "${list[@]}"; do
    echo "$item"
done
echo ""

# note that the colon. the last item will be "e:" rather than "e", other items are unaffected.
for item in "${list[@]}":
do
    echo "$item"
done
echo ""
