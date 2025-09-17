#include<iostream>
// m  ,  <>
//template 的使用
using namespace std;
void swapInt(int &a, int &b){
  int temp = a;
  a = b;
  b = temp;
}
void swapDouble(double &a, double &b){
  double temp = a;
  a = b;
  b = temp;
}

// T代表通用数据类型，紧接着后面的代码，出现T，代表通用数据类型
// 写法1、自动类型推导，推导出的数据类型，必须一致才可以使用
template<typename T> 
void newSwap(T &a , T &b){
  T temp = a;
  a = b;
  b = temp;
}

// 模板不能单独使用，务必要指定出T才可以使用
template<typename T>
void newSwap2() {

}

int main()
{
  int a = 1;
  int b = 2;
  double c2 = 3.2;
  double d2 = 4.2;

  //写法1：自动类型推导
  // swapInt(a , b);
  // cout << "execute:swapInt(a , b): \n a =  "<< a << "\n b =  "<< b  << endl;
  // newSwap(c2 , d2);
  // cout << "execute:newSwap(c2 , d2): \n c2 =  "<< c2 << "\n d2 =  "<< d2  << endl;
  // newSwap(a , d2);  // 类型务必一致才可以

  //写法2：显示指定类型
  newSwap<int>(a , b);
  cout << "execute:newSwap<int>(a , b): \n a =  "<< a << "\n b =  "<< b  << endl;
  newSwap<double>(c2 , d2);
  cout << "execute:newSwap<double>(c2 , d2): \n c2 =  "<< c2 << "\n d2 =  "<< d2  << endl;

  // newSwap2(); // 自动类型推导写法如果不在函数中指定T，就无法调用函数
  newSwap2<int>();// 显示指定类型写法指定了T的类型，才可以调用





  return 0;
}

/*
c++提供了函数模板(function template.)所谓函数模板，实际上是建立一个通用函数，其函数类型和形参类型不具体制定，用一个虚拟的类型来代表。这个通用函数就成为函数模板。凡是函数体相同的函数都可以用这个模板代替，不必定义多个函数，只需在模板中定义一次即可。在调用函数时系统会根据实参的类型来取代模板中的虚拟类型，从而实现不同函数的功能。泛型编程
    • c++提供两种模板机制:函数模板和类模板
    • 类属 - 类型参数化，又称参数模板
总结：
    • 模板把函数或类要处理的数据类型参数化，表现为参数的多态性，成为类属。
    • 模板用于表达逻辑结构相同，但具体数据元素类型不同的数据对象的通用行为。
*/