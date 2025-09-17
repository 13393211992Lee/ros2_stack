#include<iostream>
using namespace std;
// m  ,  <>
// 模板类碰到友元函数


// 前向声明模板类
template<class T1, class T2>
class Students12;

// 前向声明友元函数模板
template<class T1, class T2>
void printStu(Students12<T1, T2>& stu);


template<class T1,class T2>
class Students12 {

public:
	Students12(T1 name,T2 age) {
		this->m_name = name;
		this->m_age = age;
	}
	// 类中声明友元。类外定义，需要设置空模板参数列表
	friend void printStu<>(Students12<T1, T2>& stu); 
private:
	T1 m_name;
	T2 m_age;
};

// 友元函数模板实现
template<class T1, class T2>
void printStu(Students12<T1, T2>& stu) {
	cout << "姓名：" << stu.m_name << endl;
	cout << "年龄：" << stu.m_age << endl;
}

int main()
{
  Students12<string, int> stu("张三",11.1);

	// 类中友元函数,在类外调用的时候，直接按照普通函数的调用方式即可，不用实例化后再调用
	printStu(stu);
  return 0;
}
