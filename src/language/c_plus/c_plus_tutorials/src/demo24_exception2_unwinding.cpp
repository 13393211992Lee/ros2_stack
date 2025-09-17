#include<iostream>
using namespace std;
// m  ,  <>
//栈解旋(unwinding)
class Person{
  public:
    string mName;
    Person(){
      cout << " Person()" << endl;
    }
    Person(string name){
      cout << " Person(string name)" << endl;
    }
    Person(string name, int age){
      cout << " Person(string name, int age)" << endl;
    }
    ~Person(){
      cout << " ~Person()" << endl;
    }
};

void testFunc(){
  Person p1();
	Person p2("bbb");
	Person p3("ccc");
	Person p4("ddd", 3);

  //抛出异常
	throw 10;
}
int main()
{
  try{
    testFunc();
  }
  catch(...){
    cout<< "exception catched!" << endl;
  }
  system("pause");
  return 0;
}

/*
栈解旋顺序：
  ~p4 -> ~p3 -> ~p2 -> ~p1
  每个对象都使用同一个析构函数
*/

/*
异常被抛出后，从进入try块起，到异常被抛掷前，这期间在栈上构造的所有对象，都会被自动析构。
析构的顺序与构造的顺序相反，这一过程称为栈的解旋(unwinding).
*/

/*

构造函数可以有多个：用于不同初始化场景
析构函数只能有一个：用于统一清理资源
一对多的关系：一个析构函数处理所有构造函数创建的对象

*/