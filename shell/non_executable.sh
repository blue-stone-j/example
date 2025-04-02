# -type f: ensures only files are targeted (not directories).
# chmod a-x: removes the executable bit for all users (owner, group, others).
# {}: is a placeholder for the file found.
# +: means chmod will be called with multiple files at once for better efficiency.

find ./my_dir -type f -exec chmod a-x {} +
