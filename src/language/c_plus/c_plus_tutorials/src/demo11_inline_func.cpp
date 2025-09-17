#include<iostream>
using namespace std;
//构造函数 和 析构函数


class Person
{
private:
public:
  char* pName;
	int mTall;
	int mMoney;
  Person();
  ~Person();
};

Person::Person()
{
  cout << "构造函数调用!" << endl;
  pName = (char*)malloc(sizeof("John"));
  // strcpy(pName, "John");
  mTall = 150;
  mMoney = 100;
}

Person::~Person()
{
  cout << "析构函数调用!" << endl;
  if (pName != NULL){
    free(pName);
    pName = NULL;
  }
}

int main()
{
  class Person p;
  cout << p.pName << endl;

}

/*
构造函数主要作用在于创建对象时为对象的成员属性赋值，构造函数由编译器自动调用，无须手动调用。
析构函数主要用于对象销毁前系统自动调用，执行一些清理工作。
构造函数语法：
    • 构造函数函数名和类名相同，没有返回值，不能有void，但可以有参数。
    • ClassName(){}

析构函数语法：
    • 析构函数函数名是在类名前面加”~”组成,没有返回值，不能有void,不能有参数，不能重载。
    • ~ClassName(){}


*/


/*
内联函数
1.  在普通函数(非成员函数)函数前面加上inline关键字使之成为内联函数。
    但是必须注意必须函数体和声明结合在一起，否则编译器将它作为普通函数来对待。
    eg: inline void func(int a); // 无效
    eg: inline int func(int a){return ++;} //正确

2.  为了定义内联函数，通常必须在函数定义前面放一个inline关键字。但是在类内部定义内联函数时并不是必须的。
    任何在类内部定义的函数自动成为内联函数。

c++内联编译会有一些限制，以下情况编译器可能考虑不会将函数进行内联编译：
    • 不能存在任何形式的循环语句
    • 不能存在过多的条件判断语句
    • 函数体不能过于庞大
    • 不能对函数进行取址操作

*/