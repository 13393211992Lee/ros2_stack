#include<iostream>
using namespace std;
// m  ,  <>
// 类模板头文件和源文件分离问题

template<class T1,class T2>
class Students10 {
public:
	T1 m_name;
	T2 m_age;
	Students10(T1 name, T2 age);
	void show();
};

// 无论函数中是否用到T1、T2 都要写在类名后面的尖括号中
template<class T1, class T2>
Students10<T1, T2>::Students10(T1 name, T2 age) {
	this->m_name = name;
	this->m_age = age;
}

template<class T1, class T2>
void Students10<T1, T2>::show() {
	cout << "姓名：" << this->m_name << " 年龄：" << this->m_age << endl;
}
