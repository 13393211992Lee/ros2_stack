#include<iostream>
using namespace std;

// 静态成员函数

class Student
{
  private:
  public:
    int s_c;
    static int s_d;
    static void func1(){
      cout << "func1 execute " << endl;
      // s_c = 10; // 静态成员函数，不可以访问非静态成员变量
      s_d = 15; // 静态成员函数，可以访问静态成员变量
      
    }
};
int Student::s_d = 10;

int main()
{
  Student stu;
  stu.func1();    //调用方法1：通过对象
  Student::func1();   //调用方法2：通过类名
  return 0;
}


/*

在类定义中，前面有static说明的成员函数称为静态成员函数。静态成员函数使用方式和静态变量一样，
同样在对象没有创建前，即可通过类名调用。静态成员函数主要为了访问静态变量。不能访问普通成员变量。

*/