#include<iostream>
using namespace std;

class Base08_1 {
public:
	int m_a;
	Base08_1() {
		this->m_a = 10;
	}
};

class Base08_2 {
public:
	int m_a;
	Base08_2() {
		this->m_a = 20;
	}
};

// 多继承
class Son08 : public Base08_1,public Base08_2 {
public:
	int m_c;
	int m_d;
};

int main()
{
  Son08 s1;
	//cout << "s1大小为：" << sizeof(s1) << endl; // 16字节

  // 当多继承的父类中有同名成员，需要加作用域区分
	cout << s1.Base08_1::m_a << endl;
	cout << s1.Base08_2::m_a << endl;
  return EXIT_SUCCESS;
}
