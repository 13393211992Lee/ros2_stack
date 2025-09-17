#include<iostream>
using namespace std;

// 静态成员变量
//特点：
//1、在编译阶段就分配了内存空间
//2、类内声明，在类外进行初始化
//3、所有对象共享一份静态成员数据
class Student
{
private:
  /* data */
public:
  static int s_a; // 静态成员变量
 
};

int Student::s_a = 10;   //在类外进行初始化
int main()
{
  // 写法1：通过对象进行访问
  Student stu;
  cout << "stu.s_a static : "<< stu.s_a << endl;

  Student stu2;
  stu2.s_a = 11;
  cout << "stu.s_a 赋值: "<< stu.s_a << endl;


  //写法2：通过类名进行访问
	cout<< "Student::s_a: " << Student::s_a << endl;
  return 0;
}
