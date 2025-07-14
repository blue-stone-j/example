
# $1: first argument
# $2: second argument
pass_arguments() {
  local arg1="$1"
  local arg2="$2"
  echo "Argument 1: $arg1"
  echo "Argument 2: $arg2"
}
pass_arguments value1 value2

# $@: iterate over all passed arguments
print_all_arguments() {
    for arg in "$@"; do
        echo "$arg"
    done
}
print_all_arguments "apple" "banana" "cherry"

# pass argument and return a value
return_value() {
    echo "Hello, $1"
}
returned_value=$(return_value "World")
echo "$returned_value"