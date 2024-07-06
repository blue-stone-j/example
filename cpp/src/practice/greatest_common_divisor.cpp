#include <iostream>

int MAX_CF(int a,int b)//最大公约数
{
	int temp=0;
	while(b!=0)
	{
		temp=a%b;//取余
		a=b;//交换
		b=temp;
	}
	return a;//返回目标值
} 
int MIN_CD(int u,int v,int h)//最小公倍数
{
	return(u*v/h);
}
int main()
{
	int num1,num2;//两个数
	int cf,cd;//公约和公倍数
	std::cout<<"-------求最大公约数和最小公倍数--------"<<std::endl;
	std::cout<<"请输入两个正整数:";
	std::cin>>num1>>num2;
	cf=MAX_CF(num1,num2);
	std::cout<<"最大公约数："<<cf<<std::endl;
	cd=MIN_CD(num1,num2,cf);
	std::cout<<"最小公倍数："<<cd<<std::endl;

    return 0;
}