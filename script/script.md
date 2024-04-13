
### 01 录制特定时间的bag包
如果文件夹存在不执行操作，不存在就创建文件夹; 计算该文件夹内的文件个数(不包括文件夹)。
* `ls -l`,统计当前文件夹,不包括子文件夹; `ls -lR`, 统计当前文件夹,包括子文件夹;
* `"^-"`,统计文件个数; `"^d"`,统计文件夹个数
* `wc -l`, 统计输出信息的行数，因为已经过滤得只剩一般文件了，所以统计结果就是一般文件信息的行数，又由于一行信息对应一个文件，所以也就是文件

```Bash
mkdir -p cloud_rtk
cd cloud_rtk

num=`ls -l |grep "^-"|wc -l`

rosbag record --duration=2 -o $num /lidar1/points
```


### 02 远程下载或上传文件
* 首行是标记使用的脚本的类型，此处为`expect`脚本，也可使用`bash`脚本。
* spawn: 执行一个命令
* `expect EOF` means that all interacts are finished and return to original terminal/user. also `eof`
* send: 用于向进程发送字符串. Quotation `""` can't be omitted and must be English punctuation

```Bash
#! /usr/bin/expect

set timeout 30 # 设置超时时间, unit is second

spawn scp source_filepath destination_filepath # fork一个子进程执行scp

expect "password:" # 捕获到密码; note the colon is Chinese or English

send "password\n" # 输入密码并回车
expect EOF
```

### 03 远程连接并执行命令
* `sleep`是指停止1秒后再继续执行后面的脚本
* `send`后的字符串末尾的`\n`是转义字符，相当于输入命令后的回车键。
* 末尾处的`exit`表示退出此次登录, it's executed by the user we login。
* `interact` means that stay at current user. For example, if we omit `exit`, we will stay at the user we login.

```Bash
#!/usr/bin/expect

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
```

### 04 执行任务并定时关闭
首行是标记使用的脚本的类型，此处为`bash`脚本。<br />
两个命令间用`&`连接，前面的命令执行后挂在后台并立即开始执行后面的命令。用`&&`连接，只有前面的命令执行完毕并退出后才会执行后面的命令。<br />
第一行开启`roscore`后，计时两秒后结束`roscore`。`kill -2`相当于`Ctrl+C`。`-2`就是`Ctrl+C`发出的`siginit`。
```Bash
#!/bin/bash
roscore &
sleep 3 && kill -2
```

### 05 解压文件名有规律的系列文件
这一系列的文件名为“17.7z”、“18.7z”...“21.7z”。解压密码为“blue”。
```Bash
#!/bin/bash

set -u

for i in {17..21}
do
  filename=$i".7z"
  7z x $filename -pblue
done
```
解压所在文件夹内的所有以“.7z”结尾的文件，解压密码为“blue”。
```Bash
ls | while read line
do
  file=$line
  if echo $file | grep -q -E '\.7z$'
  then
    echo $file
    7z x $file -pblue
  fi
done
```

### 06 内嵌expect脚本并检查IP地址连通性
* ping [-dfnqrRv][-c<完成次数>][-i<间隔秒数>][-I<网络界面>][-l<前置载入>][-p<范本样式>][-s<数据包大小>][-t<存活数值>][主机名称或IP地址]
* [参考链接](https://www.runoob.com/linux/linux-comm-ping.html)  

```Bash
ip="11.11.11.11"
if ping -c 1 -w 1 ip >/dev/null;then
echo "可以连接，继续检测"
/usr/bin/expect<<-EOF   #shell中使用expect
spawn git push
expect {
  "yes/no" {
    send "yes\n"
    expect "password:"
    send "$password\n"
  }
  "password:" {
    send "$password\n"
  }
}
interact
expect eof
EOF

else
  echo "无法连接"$ip
fi
```

### 07 修改某文件夹下的特定文件
遍历所在指定文件夹及其所有子文件夹中的markdown文档，并把第四行修改为<日期+时间+时区>
```bash
#!/bin/bash

files=$(find "." -type f)

# 遍历文件
for file in $files; do
    # 检查文件名是否符合特定首尾条件
    if [[ "$file" == *_posts* && "$file" == *.md ]]; then
        # 获取文件的最近修改时间
        last_modified=$(stat -c "%Y" "$file")

        # 将时间戳转换为日月年时分秒的格式
        last_modified_formatted=$(date -d @"$last_modified" +"%Y-%m-%d %H:%M:%S %z")

        # 替换文件的第四行为最近修改时间
        sed -i "4s/.*/date:   $last_modified_formatted/" "$file"

        echo "Updated date in $file to $last_modified_formatted"
    fi
done

```

### 09 批量删除某个名称的文件夹
```Bash
find . -type d -name .git -prune -exec rm -r {} \;
```
* `.`: 指定查找的范围，即在呢个文件夹内查找
* `-name`: find命令的参数，后面接要查找的文件夹名称
* `.git`: 我们想要删除的文件夹的名称，此处我想删除的文件夹为“.git”
* `-exec`: 
* `{}`: can be read as "for each matching file/ folder" 
* `\;` is a terminator for the `-exec` clause

下面的这个命令也可以使用。
```Bash
rm -rf `find . -type d -name a`
```