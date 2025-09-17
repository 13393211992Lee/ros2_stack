#include<iostream>
using namespace std;

class Base07 {
public:
	static int m_a; // 静态成员，类内定义
	static void fun() {
		cout << "base中的fun" << endl;
	}
	static void fun(int a) {
    (void)a;
		cout << "base中的fun(int a)" << endl;
	}

};
int Base07::m_a = 10; // 静态成员，类外初始化

class Son07 : public Base07 {
public:
	static int m_a; // 静态成员，类内定义
	static void fun() {
		cout << "son中的fun" << endl;
	}
};
int Son07::m_a = 20; // 静态成员，类外初始化

int main()
{
  Son07 s1; 
  //静态成员
	// 1、通过对象访问
  cout << "静态成员" << endl;
	cout << "1、通过对象访问" << endl;
	cout << "m_a:" << s1.m_a << endl;
	cout << "父类中的m_a:" << s1.Base07::m_a << endl;

  // 2、通过类名访问
	cout << "2、通过类名访问" << endl;
	cout << "Son07::m_a  =" <<Son07::m_a << endl;
	cout << "Son07::Base07::m_a = " <<Son07::Base07::m_a<< endl;
	cout << "Base07::m_a = " <<Base07::m_a<< endl;

  //静态成员函数
  // 1、通过对象访问
	cout << "静态成员函数" << endl;
	cout << "1、通过对象访问" << endl;
  s1.fun();
  s1.Base07::fun();

  // 2、通过类名访问
	cout << "2、通过类名访问" << endl;
  Son07::fun();
  Son07::Base07::fun();


  return 0;
}

/*
继承中的静态成员特性
静态成员函数和非静态成员函数的共同点:
    1. 他们都可以被继承到派生类中。
    2. 如果重新定义一个静态成员函数，所有在基类中的其他重载函数会被隐藏。
    3. 如果我们改变基类中一个函数的特征，所有使用该函数名的基类版本都会被隐藏。
 */