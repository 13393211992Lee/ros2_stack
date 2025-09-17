#include<iostream>
using namespace std;
#include <string>

class Home10 {
	friend class Good_friend10;   // 设置类为友元

	// 不推荐这样写！后续有办法解决
	// friend void Good_friend10::visit();  // 设置类中成员函数为友元

public:
	string living_room;
	Home10() {
		this->living_room = "客厅";
		this->bed_room = "卧室";
	}

private:
	string bed_room;
};


class Good_friend10 {
public:
	Home10* m_home;  // 类的嵌套
	Good_friend10() {
		this->m_home = new Home10;// 初始化成员
	}
	void visit() {
		cout << "好朋友要去我的" << this->m_home->living_room << endl;
		cout << "好朋友要去我的" << this->m_home->bed_room << endl;
	}


};

int main(void)
{
	Good_friend10 f1;
	f1.visit();
	return EXIT_SUCCESS;
}

/*
友元语法:
• friend关键字只出现在声明处
• 其他类、类成员函数、全局函数都可声明为友元
• 友元函数不是类的成员，不带this指针
• 友元函数可访问对象任意成员属性，包括私有属性
*/

/*
友元类注意]
  1． 友元关系不能被继承。
  2． 友元关系是单向的，类A是类B的朋友，但类B不一定是类A的朋友。
  3． 友元关系不具有传递性。类B是类A的朋友，类C是类B的朋友，但类C不一定是类A的朋友。
*/