#include<iostream>
using namespace std;
class Student
{
  public:
    int age;
    Student(int age){
      this->age = age;
      // age1 = age;
    }

    bool com_age(Student& p){
      if(age== p.age){
        return true;
      }else{
        return false;
      }
    }
    Student& add_age(Student& stu){
      this->age+=stu.age;
      return *this;
    }
};


int main()
{
  Student stu1(10);
  cout << "stu1的年龄 " << stu1.age << endl;

  Student stu2(20);
  cout << "stu2的年龄 " << stu2.age << endl;

  bool res = stu2.com_age(stu1);
  cout << "res: " << res << endl;

  // 链式编程
  stu2.add_age(stu1).add_age(stu1).add_age(stu1);
  cout << "stu2.age:  "<< stu2.age << endl;

  return 0;
}
/*
运行结果：
  $ ros2 run c_tutorials c_d18_1
  stu1的年龄 10
  stu2的年龄 20
  res: 0
  stu2.age:  50
*/

/*
this指针工作原理:

c++规定，this指针是隐含在对象成员函数内的一种指针。
当一个对象被创建后，它的每一个成员函数都含有一个系统自动生成的隐含指针this，用以保存这个对象的地址，
也就是说虽然我们没有写上this指针，编译器在编译的时候也是会加上的。
因此this也称为“指向本对象的指针”，this指针并不是对象的一部分，不会影响sizeof(对象)的结果。

this指针是C++实现封装的一种机制，它将对象和该对象调用的成员函数连接在一起，
在外部看来，每一个对象都拥有自己的函数成员。一般情况下，并不写this，而是让系统进行默认设置。
*/


/*

this指针的使用
1. 当形参和成员变量同名时，可用this指针来区分
2. 在类的非静态成员函数中返回对象本身，可使用return *this.
*/