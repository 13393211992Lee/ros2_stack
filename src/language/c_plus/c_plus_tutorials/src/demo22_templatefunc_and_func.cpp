#include<iostream>
using namespace std;
//函数模板和普通函数区别
// m  ,  <>

// 函数模板
template<typename T>
T myPlus(T a, T b){
  return a+b;
}
// 普通函数
int myPlus2(int a ,int b){
  return a+b;
}
int main()
{
  int a =1;
  int b = 2;
  // myPlus(a,  b);
  cout <<"myPlus(a,  b)  =" <<myPlus(a,  b)<< endl;
  cout <<"myPlus2(a,  b)  =" <<myPlus2(a,  b)<< endl;

  // 区别
  char c = 'a'; // a的ask码 97
  // cout <<"myPlus(a,  c)  =" <<myPlus(a,  c)<< endl;          // 自动类型推导，类型必须一致
  cout <<"myPlus<int>(a,  c)  =" <<myPlus<int>(a,  c)<< endl;   // 显示指定类型 
  cout << "myPlus2(a,  c)  =" <<myPlus2(a, c) << endl;          //普通函数会发生隐式类型转换

  char cc = myPlus<int>(a,  c);
  char cc2 = myPlus2(a,  c);
  cout <<"cc  =" << cc << endl;
  cout <<"cc2  =" << cc2 << endl;

  return 0;
}
/*
输出结果为：
  myPlus(a,  b)  =3
  myPlus2(a,  b)  =3
  myPlus<int>(a,  c)  =98
  myPlus2(a,  c)  =98
  cc  =b
  cc2  =b
*/

/*
C++中的类型转换规则。
在普通函数调用中，如果参数类型不匹配，编译器会尝试进行隐式类型转换。
例如，当调用`myPlus2(a, c)`时，`c`是`char`类型，而函数参数是`int`类型，所以`char`会被隐式转换为`int`。
这里的关键是，字符在内存中是以ASCII码的形式存储的，所以`char`类型的变量`c`（值为`'a'`）会被转换为对应的ASCII码值97，
然后与整数`a`（值为1）相加，得到98，即字符`'b'`的ASCII码，所以输出是98。
*/