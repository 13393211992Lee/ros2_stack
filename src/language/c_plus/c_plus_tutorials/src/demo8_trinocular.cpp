#include<iostream>
using namespace std;
//三目运算
void test(){
  int a = 10;
  int b = 20;
  printf("ret: %d \n",a > b ? a : b );
  cout <<"b:" << b << endl;

  (a > b ? a : b) = 100;//返回的是左值，变量的引用
	cout << "b:" << b << endl;
}
int main()
{
  test();
  return 0;
}
/*
   [左值和右值概念]
   在c++中可以放在赋值操作符左边的是左值，可以放到赋值操作符右面的是右值。
   有些变量即可以当左值，也可以当右值。
   左值为Lvalue，L代表Location，表示内存可以寻址，可以赋值。
   右值为Rvalue，R代表Read,就是可以知道它的值。
   比如:int temp = 10; temp在内存中有地址，10没有，但是可以Read到它的值。
*/