// 实例116  错误的模糊引用（类继承问题）

#include <iostream>

class human // 人类
{
 public:
  bool getBeauty()
  {
    return m_beauty;
  }
  bool m_beauty;
};
class Chinese : virtual public human // 中国人
{
};
class woman : virtual public human // 女人
{
};
class me : public Chinese, public woman
{
 public:
  me(bool a)
  {
    m_beauty = a;
  }
};

int main()
{
  me _me(true);
  std::cout << _me.getBeauty() << std::endl;
  return 0;
}