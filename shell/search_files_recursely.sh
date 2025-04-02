#!/bin/bash

# 指定需要搜索的根目录，如果运行脚本时不指定目录，则默认为当前目录
root_dir="${1:-.}"

# find . -type f ! -name "*.webp" : 输出所有没有特定后缀的文件名

# 使用find命令在指定目录及其子目录下查找所有.webp文件
find "$root_dir" -type f -name "*.webp" -exec sh -c '
  for img_path; do
    # 使用dwebp工具将webp转换为png
    dwebp "$img_path" -o "${img_path%.webp}.png"
    
    # 如果转换成功,则删除原webp文件
    if [ $? -eq 0 ]; then
      rm "$img_path"
    fi
  done
' sh {} +

# |命令|解释|
# |---|---|
# |`-exec`|这个选项后跟一个命令模板，find 命令将为每个找到的文件执行这个命令。在这个脚本中，命令模板包括一个小型的 sh 脚本|
# |`sh -c`|这个命令启动一个新的shell，并执行后面单引号中的命令。这允许对每个找到的文件执行多个命令|
# |代码块' ... '|一个小型的shell脚本，它为每个传递给它的文件路径执行一系列命令|
# |`for img_path; do ... done`|这是一个循环，遍历所有传递给shell脚本的文件路径。在这个例子中，每个路径都是一个 .webp 文件|
# |`{}`|这是 find 命令的占位符，代表当前找到的文件路径。每次 find 命令找到一个符合条件的文件时，都会将这个文件的路径替换到 {} 的位置|
# |`+`|这个符号告诉 `find` 命令将多个找到的文件路径聚集起来，一次调用 -exec 后的命令来处理这些路径，而不是对每个文件单独调用一次命令。这提高了效率，因为它减少了需要启动的shell的数量|
# |`dwebp`|这是一个命令行工具，用于将 .webp 格式的图像转换为其他格式（如 PNG）|
# |`-o`|这是 dwebp 命令的一个选项，指定输出文件的路径|
# |`${img_path%.webp}.png`|这是一个参数扩展，用于生成输出文件的名称。它将当前文件路径中的 .webp 后缀替换为 .png|
# |`if [ $? -eq 0 ]; then ... fi`|这是一个条件语句，检查上一个命令（在这个例子中是 dwebp 命令）的退出状态。如果 dwebp 命令成功执行（返回值为 0），则删除原始的 .webp 文件|
# |`rm "$img_path"`|这是一个命令，用于删除指定的文件。在这个例子中，它删除原始的 .webp 文件|
# |`$?`|这是一个特殊变量，表示上一个命令的退出状态。0 通常表示成功，非零值表示失败|

# 这个脚本的作用是查找指定目录及其子目录下的所有 .webp 文件，将它们转换为 .png 格式，并在转换成功后删除原始的 .webp 文件