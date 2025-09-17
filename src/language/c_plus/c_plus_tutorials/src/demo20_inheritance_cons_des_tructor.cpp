#include<iostream>
using namespace std;

using namespace std;

class Base04 {
public:
	Base04() {
		cout << "父类的构造" << endl;
	}
	~Base04() {
		cout << "父类的析构" << endl;
	}
};

class Other {
public:
	Other() {
		cout << "Other类的构造" << endl;
	}
	~Other() {
		cout << "Other类的析构" << endl;
	}
};

class Son04 : public Base04 {
public:
	Son04() {
		cout << "子类的构造" << endl;
	}
	~Son04() {
		cout << "子类的析构" << endl;
	}
	Other oth;
};
int main()
{
	Son04 s1; // 子类中有其他类的成员； 先调用父类构造，再是子类中的其他成员对象的构造，最后是子类中的构造，析构相反
  return EXIT_SUCCESS;
}

/*
构造函数和析构函数的执行顺序：
构造： base others child
析构： child others base

执行结果：
  父类的构造
  Other类的构造
  子类的构造
  子类的析构
  Other类的析构
  父类的析构

*/