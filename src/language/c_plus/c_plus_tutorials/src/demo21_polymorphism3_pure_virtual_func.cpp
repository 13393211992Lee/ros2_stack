#include<iostream>
using namespace std;
class Cal{ 		 // 类中有纯虚函数，这个类也叫做抽象类，无法实现实例化
public:
	int m_a;
	int m_b;	

	// 虚函数
	// virtual int getRes() {
	// 	return 0;
	// }

	// 纯虚函数
	// 纯虚函数使用关键字virtual，并在其后面加上=0。如果试图去实例化一个抽象类，编译器则会阻止这种操作。
	virtual int getRes() = 0;
};

// 加法
class Add : public Cal {
public:
	int getRes() {
		return m_a + m_b;
	}
};

// 减法
class Sub : public Cal {
public:
	int getRes() {
		return m_a - m_b;
	}
};

// 抽象类的子类务必要重写父类中的纯虚函数，否则也属于抽象类，无法实现实例化
class Test : public Cal {
public:

};
int main(void)
{
	// 多态可以改善代码的可读性和组织性，同时也可以让程序具有可扩展性
	// 动态多态产生条件：
	// 1、要有继承关系
	// 2、父类中有虚函数、子类要重写父类的虚函数
	// 3、父类的指针或引用指向子类的对象

	//Cal* c1 = new Add;
	//c1->m_a = 1;
	//c1->m_b = 2;
	//cout << c1->getRes() << endl;

	//Cal* c1 = new Sub;
	//c1->m_a = 1;
	//c1->m_b = 2;
	//cout << c1->getRes() << endl;


	Add a1;
	Cal& c1 = a1;
	c1.m_a = 1;
	c1.m_b = 2;
	cout << c1.getRes() << endl;


	Sub s1;
	Cal& c2 = s1;
	c2.m_a = 1;
	c2.m_b = 2;
	cout << c2.getRes() << endl;

	// 类中有纯虚函数，这个类也叫做抽象类，无法实现实例化
	//Cal c1;

	// 抽象类的子类务必要重写父类中的纯虚函数，否则也属于抽象类，无法实现实例化
	// Test t1;
	return EXIT_SUCCESS;
}
