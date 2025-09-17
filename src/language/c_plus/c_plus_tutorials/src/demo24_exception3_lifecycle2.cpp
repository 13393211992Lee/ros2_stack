#include<iostream>
using namespace std;
// m  ,  <>
// 异常变量生命周期

class MyExpetion {
public:
	MyExpetion() {
		cout << "默认构造函数" << endl;
	}
	MyExpetion(const MyExpetion& e) {
		cout << "复制构造函数" << endl;
	}
	~MyExpetion() {
		cout << "析构函数" << endl;
	}
};
void doThrow(){
	throw MyExpetion();   // 抛出匿名对象 
}
int main()
{
  try {
		doThrow();
	}catch(MyExpetion& e){
	// MyExpetion& e 引用传递，不会执行复制构造函数，效率高，推荐使用！
    	cout << "自定义的类型MyExpetion e捕获" << endl;
  }
  return EXIT_SUCCESS;
}


/*

构造函数可以有多个：用于不同初始化场景
析构函数只能有一个：用于统一清理资源
一对多的关系：一个析构函数处理所有构造函数创建的对象

*/
