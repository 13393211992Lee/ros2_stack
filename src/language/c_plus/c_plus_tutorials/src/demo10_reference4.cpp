#include<iostream>
using namespace std;
//发现是引用，转换为 int* const ref = &a;
void testFunc(int& ref){
  ref = 100;  // ref是引用，转换为*ref = 100
}


int main()
{
  int a = 10;
  int& ref_a = a;  //自动转换为 int* const aRef = &a;这也能说明引用为什么必须初始化
  ref_a = 20;       //内部发现aRef是引用，自动帮我们转换为: *aRef = 20;
  cout << "a: " << a << endl;
  cout << "ref_a: " << ref_a << endl;
  testFunc(a);
  cout << "a: " << a << endl;

  return 0;
}
/*
引用的本质
引用的本质在c++内部实现是一个指针常量.
Type& ref = val; // Type* const ref = &val;

c++编译器在编译过程中使用常指针作为引用的内部实现，因此引用所占用的空间大小与指针相同，只是这个过程是编译器内部实现，用户不可见。
*/