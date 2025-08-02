

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

import numpy as np
# convert ndarray to list
arr = np.array([[1, 2, 3], [4, 5, 6]])
lst = arr.tolist()

'''dictionary'''
dict = {} # Declare an empty dictionary
dict['key1'] = 'value1'  # Add a key-value pair
dict1 = {'a': 1, 'b': 2}
dict2 = {'c': 3, 'd': 4}
merged_dict = {**dict1, **dict2}
print(merged_dict)

'''tuple'''