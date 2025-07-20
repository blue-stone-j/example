#include <iostream>
#include <set>
#include <map>
#include <string>
#include <stack>

int main()
{
  std::multiset<short> multi_set; // 创建multiset容器对象
  multi_set.insert(1);
  multi_set.insert(1);
  multi_set.insert(2);

  std::multimap<std::string, std::string> multi_map1; // 创建multimap容器对象
  multi_map1.insert(std::pair<std::string, std::string>("1", "1"));
  multi_map1.insert(std::pair<std::string, std::string>("1", "2"));


  std::multiset<short>::iterator mul;
  std::multimap<std::string, std::string>::iterator it;

  std::cout << "multiset容器中各元素的值:";
  for (mul = multi_set.begin(); mul != multi_set.end(); mul++)
  {
    std::cout << *mul << " ";
  }
  std::cout << std::endl;

  std::cout << "multimap容器中各元素的值:";
  for (it = multi_map1.begin(); it != multi_map1.end(); ++it)
  {
    std::cout << it->first << " " << it->second << " ";
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