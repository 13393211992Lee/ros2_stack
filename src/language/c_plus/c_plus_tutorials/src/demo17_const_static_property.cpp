#include<iostream>
using namespace std;

class Student
{
private:

public:
  static int share1;          
  const static int share = 20;   //只读区，不可修改

};

int share1 = 120;



int main()
{
  cout << Student::share << endl;   
  // Student::share = 100;          //const修饰 不可修改

  cout << Student::share1 << endl;    //
  Student::share1 = 200;
  cout << Student::share1 << endl;

  return 0;
}


/*
如果一个类的成员，既要实现共享(static)，又要实现不可改变(const)，那就用 static const 修饰。
定义静态const数据成员时，最好在类内部初始化。避免在类外部重复初始化，也为了代码的可读性和可维护性。

const 不可修改
static 可以共享 类直接访问
*/
