#include<iostream>
using namespace std;
// m  ,  <>

// 类模板
// template<class NAMETYPE, class AGETYPE>
template<class NAMETYPE, class AGETYPE = int>
class Student{
public:
  NAMETYPE a_name;
  AGETYPE a_age;
  Student(NAMETYPE name, AGETYPE age ){
    this->a_name = name;
    this->a_age = age;
  }
  void show(){
    cout << "姓名：" << this->a_name << endl;
		cout << "年龄：" << this->a_age << endl;
  }
};
int main()
{
  // 1、类模板无法使用自动类型推导的写法
	// Students06 stu0("张三",18);

  //2、显示指定类型
	Student<string, int> stu1("张三", 18);

  // "template<class NAMETYPE, class AGETYPE = int>"
	// class AGETYPE = int 设置默认的类型值，后面在使用的时候，就不用再指定类型了
	Student<string> stu2("张三",18);
	stu1.show();



  return 0;
}
