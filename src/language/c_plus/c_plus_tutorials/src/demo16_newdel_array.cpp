#include<iostream>
using namespace std;

class Student
{
private:
  
public:
  Student();
  Student(int a);
  ~Student();
};

Student::Student(){
  cout << "Students 的默认构造函数" << endl;
}
Student::Student(int a){
  (void)a;
  cout << "Students 的带参构造函数" << endl;
}
Student::~Student(){
  cout << "Students 的析构函数" << endl;
}

int main()
{
  // malloc和new区别：
  //1、malloc和free属于库函数    new和delete属于运算符
  //2、malloc不会调用构造函数  new会调用构造函数
  //3、malloc返回void* 在c++需要写强转。 new 返回创建的对象的指针

  // Student* stu = new Student;
  // delete stu;   //执行析构函数

	//注意1：不要使用void*去接收new出来的对象，后续delete的时候，就无法找到
	// void* p = new Student;
	// delete p;  // 无法调用析构函数

  //注意：2: 利用new 开辟数组
	//int* p_i = new int[10];
	//double* p_d = new double[10];

  // 堆空间开辟数组，会调用默认构造函数10次,至少要保证有默认构造函数的存在才不会报错
	// Student* p = new Student[10];
	// delete [] p;  // 删除堆空间数组的时候，需要在delete后面添加[] 也会执行析构函数10次

  // 栈上开辟数组
	Student arr[5] = { Student(),Student(2),Student(3),Student(4),Student(5) };
  return 0;
}
