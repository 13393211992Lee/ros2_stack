#include<iostream>
// 静态成员实现单例模式
using namespace std;

class King {
	// 公共的函数，为了让外部可以获取唯一的实例
public:
	// getInstance约定俗成
	static King* getInstance() {
		return true_king;
	}


private:
	// 构造函数设置为私有，无法直接从外部创建类的对象
	// 只能在类内进行调用
	King() {
		
	};
	// 默认的拷贝构造函数要覆盖重写，让它变成私有的，就可以把这个漏洞补上了
	King(const King &k1) {
		
	}


	// 私有的静态指针，用于存储类的唯一实例
	static King* true_king;  // 静态成员：类内声明，类外部初始化
};

// 类外初始化静态成员
King* King::true_king = new King;

int main(void)
{
	// 构造函数设置为私有，无法直接从外部创建类的对象
	//King k1;
	//King* k2 = new King;

	King* k1 = King::getInstance();
	King* k2 = King::getInstance();
	// 默认的拷贝构造函数要覆盖重写，让它变成私有的，就可以把这个漏洞补上了
	// King* k3 = new King(*k1);

	if (k1 == k2) {
		cout << "k1=k2" << endl;
	}
	else {
		cout << "k1！=k2" << endl;
	}
	// if (k1 == k3) {
	// 	cout << "k1=k3" << endl;
	// }
	// else {
	// 	cout << "k1!=k3" << endl;
	// }
	return EXIT_SUCCESS;
}
/*
单例模式是一种常用的软件设计模式。在它的核心结构中只包含一个被称为单例的特殊类。
通过单例模式可以保证系统中一个类只有一个实例而且该实例易于外界访问，从而方便对实例个数的控制并节约系统资源。
如果希望在系统中某个类的对象只能存在一个，单例模式是最好的解决方案。
*/