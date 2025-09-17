#include<iostream>
using namespace std;
// m  ,  <>
//类模板做函数参数

template<class T1 , class T2>
class Student{
public:
  T1 m_name;
  T2 m_age;
  Student(T1 name, T2 age){
    this->m_name = name;
    this->m_age = age;
  }
  void show(){
    cout << "姓名：" << this->m_name << endl;
		cout << "年龄：" << this->m_age << endl;
  }
};
  // 1、指定传入类型：使用最广泛
  void sendVal1(Student<string, int> &stu) {
    stu.show();
  }

  // 2、参数模板化
  template<class T1,class T2>
  void sendVal2(Student<T1, T2> &stu) {

    cout << "T1类型是：" << typeid(T1).name() << endl;
    cout << "T2类型是：" << typeid(T2).name() << endl;
    stu.show();
  }

  // 3、整个类 模板化
  template<class T>
  void sendVal3(T &stu) {
    // 查看类型  typeid(T).name()
    cout << "T类型是：" << typeid(T).name() << endl;
    stu.show();
  }

int main()
{
    // 1、指定传入类型：使用最广泛
  Student<string , int> stu("cici", 2);
  sendVal1(stu);

  // 2、参数模板化
  sendVal2(stu);

  // 3、整个类 模板化
  sendVal3(stu);

  return 0;
}
