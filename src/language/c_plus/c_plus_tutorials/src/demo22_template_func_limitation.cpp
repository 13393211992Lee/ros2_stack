#include<iostream>
using namespace std;
// m  ,  <>
// 模板的局限性

class Student{
public:
  string m_name;
  int m_age;
  Student(string name , int age);
};
Student::Student(string name, int age){
  this->m_name = name;
  this->m_age = age;
}

template<typename T>
bool myCompare(T &a , T &b){
  if(a == b){
    return true;
  }else{
    return false;
  }
}
// 利用具体化，实现对于自定义的类型进 行对比 需要提供特殊模板
template<> bool myCompare(Student& a , Student& b){
  if(a.m_age == b.m_age && a.m_name == b.m_name){
    return true;
  }else{
    return false;
  }
}
int main()
{
  int a = 2;
  int b = 2;
  bool res = myCompare(a, b);
  if(res){
    cout << "a = b " << endl;
  }else{
    cout << "a != b " << endl;
  }

  Student stu1("xusir",39);
  Student stu2("xusir",39);
  bool res2 = myCompare(stu1, stu2);
  if(res2){
    cout << "stu1 = stu2 " << endl;
  }else{
    cout << "stu1 != stu2 " << endl;
  }
  return 0;
}

/*

假设有如下模板函数：
	template<class T>
	void f(T a, T b)
	{ … }
  
如果代码实现时定义了赋值操作 a = b，但是T为数组，这种假设就不成立了
同样，如果里面的语句为判断语句 if(a>b),但T如果是结构体，该假设也不成立，
另外如果是传入的数组，数组名为地址，因此它比较的是地址，而这也不是我们所希望的操作。
总之，编写的模板函数很可能无法处理某些类型，另一方面，有时候通用化是有意义的，但C++语法不允许这样做。
为了解决这种问题，可以提供模板的重载，为这些特定的类型提供具体化的模板。
*/