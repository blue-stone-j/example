ip="11.11.11.11"

# * ping [-dfnqrRv][-c<完成次数>][-i<间隔秒数>][-I<网络界面>][-l<前置载入>][-p<范本样式>][-s<数据包大小>][-t<存活数值>][主机名称或IP地址]
# * [参考链接](https://www.runoob.com/linux/linux-comm-ping.html)  
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

if ping -c 1 -w 1 ${ip} >/dev/null;then
  echo ${ip}
  /usr/bin/expect<<-EOF   #shell中使用expect
    set timeout -1
    spawn ssh $username@${ip}
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
    
    sleep 1
    
    send "cd ~/Documents\n"
    
    send "exit\n"

	expect eof
	EOF

else
  echo "无法连接"$1": "${ipmap[$1]}
fi