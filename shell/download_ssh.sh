#! /usr/bin/expect

# 首行是标记使用的脚本的类型，此处为`expect`脚本，也可使用`bash`脚本。
# spawn: 执行一个命令
# `expect EOF` means that all interacts are finished and return to original terminal/user. also `eof`
# * send: 用于向进程发送字符串. Quotation `""` can't be omitted and must be English punctuation

set timeout 30 # 设置超时时间, unit is second

spawn scp source_filepath destination_filepath # fork一个子进程执行scp

expect "password:" # 捕获到密码; note the colon is Chinese or English

send "password\n" # 输入密码并回车
expect EOF