#include<iostream>
using namespace std;

class Base03 {
public:
	int m_a;
protected:
	int m_b;
private:
	int m_c; // 哪怕是私有属性，子类访问不到，子类会显示这个空间的占用，被编译器隐藏而已
};

class Son03 : public Base03 {
public:
	int m_d;
};

int main()
{
  cout << "son03的大小为：" << sizeof(Son03) << endl; // 16
  return 0;
}

/*
int ：通常占用 4 字节（取决于编译器和平台）。
a b c d  所以：16

*/