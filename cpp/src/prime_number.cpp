#include <iostream> 

int main() 
{
	int range_max;//最大值
	std::cout<<"--------判断素数----------"<<std::endl;
	std::cout<<"请输入最大素数检测界限值：";
	std::cin>>range_max;
	int *num=new int[range_max+1];//申请内存
	for(int i=0;i<=range_max;i++)//赋值
		num[i]=i;
	for(int j=2;j<=range_max;j++)//由大于2的值为一个因子
	{
		if(num[j]!=0)//没确定为合数
		{
			for(int k=2;k*j<=range_max;k++)//由大于2的值为另一个因子
				num[k*j]=0;//此值重新赋为0
		}
	}
	for(int n=2;n<=range_max;n++)//输出范围内所有素数
	{
		if(num[n]!=0)//不为0即为素数
		{
			static int count=0;//计数
			std::cout<<num[n]<<" ";//输出素数
			count++;
			if(count%5==0)//5个数为一行
				{std::cout<<std::endl;}
		}
	}
	std::cout<<std::endl;
	delete []num;
	num=NULL;

	return 0;
}