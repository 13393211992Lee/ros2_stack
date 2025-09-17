#include<iostream>
using namespace std;

class Cal{
public:
	int m_a;
	int m_b;
	virtual int getRes() {
		return 0;
	}
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

int main(){
// 多态可以改善代码的可读性和组织性，同时也可以让程序具有可扩展性
	// 动态多态产生条件：
	// 1、要有继承关系
	// 2、父类中有虚函数、子类要重写父类的虚函数
	// 3、父类的指针或引用指向子类的对象
	
	// Cal* c1 = new Add;
	// c1->m_a = 1;
	// c1->m_b = 2;
	// cout << c1->getRes() << endl;

	// Cal* c1 = new Sub;
	// c1->m_a = 1;
	// c1->m_b = 2;
	// cout << c1->getRes() << endl;


	Add a1;
	Cal& c1 = a1;
	c1.m_a = 1;
	c1.m_b = 2;
	cout << c1.getRes() << endl;


	// Sub s1;
	// Cal& c2 = s1;
	// c2.m_a = 1;
	// c2.m_b = 2;
	// cout << c2.getRes() << endl;
	return EXIT_SUCCESS;
}
