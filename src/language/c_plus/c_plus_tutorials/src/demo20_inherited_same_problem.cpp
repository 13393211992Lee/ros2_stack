#include<iostream>
using namespace std;

class Base06 {
public:
	int m_a;
	Base06() {
		this->m_a = 10;
	}
	void fun() {
		cout << "父类的fun函数" << endl;
	}
	void fun(int a) {
		(void)a;
		cout << "父类的fun(int a)函数" << endl;
	}
};

class Son06 : public Base06 {
public:
	int m_a;
	Son06() {
		this->m_a = 20;
	}
	void fun() {
		cout << "子类的fun函数" << endl;
	}
};

int main()
{
  Son06 s1;
	// 同名成员，就近原则 
	// cout << "s1.m_a: " << s1.m_a << endl;
	// s1.fun();

	// 同名成员 调父类成员：设置作用域即可
	// 子类中定义了和父类同名的成员函数，父类的同名成员函数都会被隐藏。可以通过作用域来调用
	cout << "s1.m_a：" << s1.Base06::m_a << endl;
	s1.Base06::fun();
  return 0;
}


/*
继承中同名成员的处理方法

• 当子类成员和父类成员同名时，子类依然从父类继承同名成员
• 如果子类有成员和父类同名，子类访问其成员默认访问子类的成员(本作用域，就近原则)
• 在子类通过作用域::进行同名成员区分(在派生类中使用基类的同名成员，显示使用类名限定符)

*/