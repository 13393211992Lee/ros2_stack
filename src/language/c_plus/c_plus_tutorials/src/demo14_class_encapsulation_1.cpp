//类的封装


#include<iostream>
using namespace std;

//封装两层含义
//1. 属性和行为合成一个整体
//2. 访问控制，现实事物本身有些属性和行为是不对外开放
// struct 和class区别：
// class默认权限是私有，struct默认权限是公共
// c++才有class，今后封装的时候就使用class即可
class Person
{
  private:
    int pwd;
  public:
    string name;
  protected:
    string car;
  public:
	void fun1() {
		// 在类的内部(作用域范围内)，没有访问权限之分，所有成员可以相互访问
		name = "张三";
		car = "奥拓";
		pwd = 111111;
		cout << "name: "<<name << endl;
		cout << "car: "<<car << endl;
		cout << "pwd: "<<pwd << endl;
	}

};



int main()
{
	struct Person p1;
	p1.name = "李四";  // 公共权限。外部可以访问
	//p1.car = "自行车";   // 保护权限,外部不可以访问
	//p1.pwd = 0; // 私有权限,外部不可以访问
	cout << "p1.name: "<<p1.name << endl;
	p1.fun1();
	
	class Person p2;
	p2.name = "王五";
	// p2.pwd = "12345";
	// p2.car = "自行车";
	cout << "p2.name: "<<p2.name << endl;

	
	return EXIT_SUCCESS;
}


/*


    • 封装
    1.  把变量（属性）和函数（操作）合成一个整体，封装在一个类中
    2.  对变量和函数进行访问控制
    • 访问权限
    1. 在类的内部(作用域范围内)，没有访问权限之分，所有成员可以相互访问
    2. 在类的外部(作用域范围外)，访问权限才有意义：public，private，protected
    3. 在类的外部，只有public修饰的成员才能被访问，在没有涉及继承与派生时，		private和protected是同等级的，外部不允许访问


	[struct和class的区别?]
	class默认访问权限为private,struct默认访问权限为public.
*/