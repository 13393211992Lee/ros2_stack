#include<iostream>
using namespace std;

//构造函数 和 析构函数
// 构造函数和其他函数不同，除了有名字，参数列表，函数体之外还有初始化列表。
// 三个方法 
class Student
{
private:
 

public:
  int s_a;
	int s_b;
	int s_c;
  Student();
  Student(int a, int b, int c);
  ~Student();
};

//普通写法
// Student::Student(int a, int b, int c)
// {
// 	s_a = a;
// 	s_b = b;
// 	s_c = c;
// }

// 初始化列表写法1：
// Student::Student():s_a(4),s_b(5),s_c(5)
// {
//   cout << "初始化函数：Student():s_a(4),s_b(5),s_c(5) " << endl;
// }

// //初始化列表写法2：
Student::Student(int a, int b, int c):s_a(a), s_b(b), s_c(c)
{
  cout << "初始化函数： Student(int a, int b, int c):s_a(a), s_b(b), s_c(c)" << endl;
}

//析构函数
Student::~Student()
{
  cout << "调用析构函数~Student()" << endl;

}

int main()
{

  // 普通写法
  // Student stu1(1 ,2 ,3);
  // cout << stu1.s_a << endl;
  // cout << stu1.s_b << endl;
  // cout << stu1.s_c << endl;  // 1 2 3 

  // 初始化列表写法1：
  // Student stu1;
  // cout << stu1.s_a << endl;
  // cout << stu1.s_b << endl;
  // cout << stu1.s_c << endl;     // 4 5 5 

  //初始化列表写法2：
  Student stu1(7,8,9);
  cout << stu1.s_a << endl;
  cout << stu1.s_b << endl;
  cout << stu1.s_c << endl;
  return 0;
}
