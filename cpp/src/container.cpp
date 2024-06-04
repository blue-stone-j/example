#include <iostream>
#include <vector>
#include <deque>
#include <list>
#include <set>
#include <map>
#include <string>
#include <stack>

int main()
{
  std::multiset<short> _mset; // 创建multiset容器对象
  _mset.insert(1);
  _mset.insert(1);
  _mset.insert(2);

  std::multimap<std::string, std::string> _mmap; // 创建
  _mmap.insert(std::pair<std::string, std::string>("1", "1"));
  _mmap.insert(std::pair<std::string, std::string>("1", "2"));


  std::multiset<short>::iterator mul;
  std::multimap<std::string, std::string>::iterator mulm;

  std::cout << "multiset容器中各元素的值：";
  for (mul = _mset.begin(); mul != _mset.end(); mul++)
  {
    std::cout << *mul << " ";
  }
  std::cout << std::endl;

  std::cout << "multimap容器中各元素的值：";
  for (mulm = _mmap.begin(); mulm != _mmap.end(); mulm++)
  {
    std::cout << mulm->first << " " << mulm->second << " ";
  }
  std::cout << std::endl;

  std::stack<int> st;

  st.push(1); // 向栈内压入元素
  st.push(2);
  st.push(3);
  st.push(4);
  while (!st.empty()) // 没到末尾
  {
    std::cout << st.top() << " "; // 输出
    st.pop();                     // pop掉
  }
  std::cout << std::endl;

  return 0;
}