#include<iostream>
#include <cstring>    // 添加此头文件以使用 strlen 和 strcpy
#include <cstdlib>    // 确保 malloc 和 free 的声明（可选，但建议）
using namespace std;
class Students05
{
private:
  /* data */
public:
  char* s_name;
	int s_age;

  Students05(const char* name,int age);
  Students05(const Students05 &p);

  ~Students05();
};
// 有参构造函数，目的是做初始化
Students05::Students05(const char* name,int age)
{
  s_name = (char *)malloc(strlen(name) + 1); // strlen(name)默认不带\0 
		strcpy(s_name,name);
		s_age = age;
};
// 拷贝构造函数
Students05::Students05(const Students05 &p){
  s_name = (char *)malloc(strlen(p.s_name) + 1);
  strcpy(s_name,p.s_name);
  s_age = p.s_age;
}

// 调用析构函数，来free堆空间
// 默认调用的编译器自动生成的拷贝构造函数，实现的是浅拷贝（值拷贝），
//这种方式两个对象使用同一块堆内存空间，所以free的时候，第二次会报错。解决方案就是自己写一个拷贝构造函数来解决
Students05::~Students05()
{
  if (s_name) {
			free(s_name);
			s_name = NULL;
		}
}

int main()
{
  Students05 stu1("张三",18);
	cout << "姓名1：" << stu1.s_name << endl;
	cout << "年龄1：" << stu1.s_age << endl;

	Students05 stu2(stu1);  // 利用默认的拷贝构造函数
	cout << "姓名2：" << stu1.s_name << endl;
	cout << "年龄2：" << stu1.s_age << endl;

  return EXIT_SUCCESS;
}
