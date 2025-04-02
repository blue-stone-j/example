mkdir -p cloud_rtk # 如果文件夹存在不执行操作，不存在就创建文件夹; 计算该文件夹内的文件个数(不包括文件夹)
cd cloud_rtk

# `ls -l`,统计当前文件夹,不包括子文件夹; `ls -lR`, 统计当前文件夹,包括子文件夹;
# `"^-"`,统计文件个数; `"^d"`,统计文件夹个数
# `wc -l`, 统计输出信息的行数，因为已经过滤得只剩一般文件了，所以统计结果就是一般文件信息的行数，又由于一行信息对应一个文件，所以也就是文件
num=`ls -l |grep "^-"|wc -l`

rosbag record --duration=2 -o $num /lidar1/points