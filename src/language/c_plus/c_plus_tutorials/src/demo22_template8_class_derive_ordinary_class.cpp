#include<iostream>
using namespace std;
// m  ,  <>
// 类模板派生普通类

template<class T>
class Base{
public:
  T a;
};

 // 务必要指定出父类中的T到底是什么数据类型，才可以给子类分配内存空间
class Son1 : public Base<int>{
};

 // 务必要指定出父类中的T到底是什么数据类型，才可以给子类分配内存空间
template<class T1 , class T2>
class Son2 : public Base<T1>{
public:
  T2 b;
  void show(){
    cout<< typeid(T1).name() <<endl;
    cout<< typeid(b).name() <<endl;
  }
};

int main()
{
  Son2<double ,int> son2;
  son2.show();// 实例化的时候才传入类型，更灵活
  return 0;
}
