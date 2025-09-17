#include<iostream>
using namespace std;

class Student
{
private:
  /* data */
public:
  // int s_a;
  static int s_b;
  double s_c;
  void func1(){}
  void func2(){}

};
// int Student::s_b = 5;

int main()
{
  Student stu;
  cout << "sizeof(stu): "<< sizeof(stu) << endl;
  return 0;
}

/*
解析：
sizeof(stu): 16 .
16由谁组成：
    int s_a：通常占用 4 字节（取决于编译器和平台）。
    double s_c：通常占用 8 字节。
    总大小：4 + 8 = 12 字节。

    内存对齐规则
    对齐要求：double 类型通常需要 8 字节对齐。
    实际存储：
    编译器会在 int s_a（4 字节）后填充 4 字节空白，以满足 double s_c 的对齐要求。
    总大小：4（s_a） + 4（填充） + 8（s_c） = 16 字节。


static int s_b 是静态成员变量，存储在全局/静态存储区，不属于对象。即使定义并初始化 s_b，sizeof(stu) 仍为 16。
func1 和 func2 是成员函数，存储在代码区。无论添加或删除成员函数，sizeof(stu) 不会改变。

总结：
非静态成员变量，算对象的存储空间  eg:s_a   s_c
成员函数，不算对象的存储空间      eg: func1() func2()
静态成员变量，不算对象的存储空间
*/