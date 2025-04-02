#!/usr/bin/expect

# `sleep`是指停止1秒后再继续执行后面的脚本
# `send`后的字符串末尾的`\n`是转义字符，相当于输入命令后的回车键。
# 末尾处的`exit`表示退出此次登录, it's executed by the user we login。
# `interact` means that stay at current user. For example, if we omit `exit`, we will stay at the user we login.

set timeout 100
set username [xxxx]
set password [123456]
set ip   [1.1.1.1]

spawn ssh -l $username $ip

expect {
    "(yes/no)?" {
        send "yes\n"
        expect "password:"
        send "$password\n"
    }
    "password:" {
        send "$password\n"
    }
}

sleep 1
send "cd /home/nnnn/bag\n"
sleep 1
send "rosbag record -a\n"
sleep 2
exit
interact