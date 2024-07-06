#include <iostream>

template <class T1, class T2>
class A // 基类A
{
 private:
  T1 X; // 成员变量T1型
  T2 Y; // 成员变量T2型
 public:
  A() {}         // 默认构造函数
  A(T1 x, T2 y); // 带参构造函数
  void show();
  ~A() {} // 析构函数
};

template <class T1, class T2>
A<T1, T2>::A(T1 x, T2 y)
{
  X = x;
  Y = y;
}
template <class T1, class T2>
void A<T1, T2>::show()
{
  std::cout << "X=" << X << "\n"
            << "Y=" << Y << std::endl;
}

template <class T1, class T2>
class B : public A<T1, T2> // 派生类B
{
 public:
  B() {}                     // 派生类默认构造函数
  B(T1 x, T2 y, T1 w, T2 l); // 带参构造函数
  void display();
  ~B() {} // 派生类析构函数
 private: // 私有成员变量
  T1 W;
  T2 H;
};
template <class T1, class T2>
B<T1, T2>::B(T1 x, T2 y, T1 w, T2 h) :
  A<T1, T2>(x, y)
{
  W = w;
  H = h;
}
template <class T1, class T2>
void B<T1, T2>::display()
{
  std::cout << "W=" << W << "\n"
            << "H=" << H << std::endl;
}
int main()
{
  B<int, double> *q = new B<int, double>(2, 3.3, 1, 2.1);
  std::cout << "派生类成员变量：" << std::endl;
  q->display();
  std::cout << "基类成员变量：" << std::endl;
  q->show();
  delete q;

  return 0;
}