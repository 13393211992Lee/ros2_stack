#include<iostream>
// const修饰成员函数（常函数）
using namespace std;
class Students08 {
public:
	int m_age;
	mutable int m_hehe; // mutable可变的。可以让这个成员函数被修改

	Students08(int age) {
		this->m_age = age;
	}

	// 常函数：修饰成员函数中的this指针，让指针指向的值不可以被修改
	void show_class() const {
		//this->m_age = 100; // 想让这句话失效
		m_hehe = 100; // mutable可变的。可以让这个成员函数被修改
		// this指针的本质： Students08* const this
		//this = NULL; // 本质this是const修饰的，无法修改

		cout << "Students08的age是: " << this->m_age << endl;
		cout << "Students08的m_hehe是: " << this->m_hehe << endl;
	}
  void func1(){
		cout << "func1 were executed..." << endl;

  }
};

int main(void)
{
  const Students08 stu(10);   //  常对象
  // stu.m_age = 10;             // 默认常对象里面的成员变量无法修改
  stu.m_hehe = 20;               // 常对象可修改 mutable 修饰的成员变量.

  // stu.func1();                   // 常对象不可以调用普通函数
  stu.show_class();               // 常对象可以调用常函数
	return EXIT_SUCCESS;
}

/*
    • 用const修饰的成员函数时，const修饰this指针指向的内存区域，成员函数体内不可以修改本类中的任何普通成员变量，
    • 当成员变量类型符前用mutable修饰时例外。
*/