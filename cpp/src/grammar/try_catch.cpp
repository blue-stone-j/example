#include <iostream>

// execute statements in try
// 如果执行的过程中没有异常拋出，skip all catch
// 如果拋出了异常，立即跳转到第一个“异常类型”和拋出的异常类型匹配的 catch 块中执行（称作异常被该 catch 块“捕获”），执行完后skip all catch
// 如果一个函数在执行过程中拋出的异常在本函数内就被 catch 块捕获并处理，那么该异常就不会拋给这个函数的调用者（也称为“上一层的函数”）；如果异常在本函数中没有被处理，则它就会被拋给上一层的函数。

double divide(double a, double b)
{
  if (b == 0) // 除数为0
  {
    throw b; // 抛出异常
  }
  return a / b; // 返回值
}
int main()
{
  try // 定义异常
  {
    std::cout << "2除以3的值为: " << divide(2, 3) << std::endl; // 除数不为0
    std::cout << "2除以0的值为: ";                              // 除数为0
    std::cout << divide(2, 0) << std::endl;
  }
  catch (double)
  {
    std::cerr << "错误: 除数为0" << std::endl; // 输出错误提示
    exit(-1);                                  // 异常退出
  }
  catch (int e)
  {
    std::cout << "catch(int) " << e << std::endl;
  }
  catch (std::string s)
  {
    throw;
  } // 继续抛出捕获的异常给上一层的函数
  catch (...)
  {
    std::cout << "catch (...)" << std::endl;
  } // 这样的 catch 块能够捕获任何还没有被捕获的异常

  return 0;
}