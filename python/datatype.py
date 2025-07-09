

'''list'''

# Declare a list
list1 = [1, 2, 3]

# Add an element to the list
list1.append(4)

# Display the list
print(list1)

list2 = [5, 6, 7]

# Merge two lists
merged = list1 + list2

# concatenate two lists using the unpacking operator
merged = [*list1, *list2]
print(merged)  # Output: [1, 2, 3, 4, 5, 6]

# merge two lists using extend
list3 = [8, 9, 10]
list3.extend(merged)

# merge two lists using itertools.chain for large lists or lazy iteration
import itertools
merged_iter = itertools.chain(list1, list2)
merged_list = list(merged_iter)
print(merged_list)

'''tuple'''