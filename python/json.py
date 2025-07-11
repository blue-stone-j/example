import json

example_dict = {
"name": "Example",
"value": 123,
"items": [1, 2, 3],
"nested": {
    "a": 10,
    "b": 20
}
}

filename = "example.json"
indent = 4  # Set to None for no indentation, or an integer for pretty printing
with open(filename, 'w', encoding='utf-8') as f:
    json.dump(example_dict, f, indent=indent, ensure_ascii=False)