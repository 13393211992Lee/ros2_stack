#include<iostream>
using namespace std;
//值传递（无法真正交换外部变量）
void ValueSwap(int m,int n ){
  int temp = m;
  m = n;
  n = temp;
  cout << "m:" << m << " n:" << n << endl;  // m:15 n:10
}

//指针传递（通过指针操作实际内存）
void PointSwap(int* m, int* n){
  int temp = *m;
  *m = *n;
  *n = temp;
  cout << "m:" << m << " n:" << n << endl;  //m:0x7ffeae2f9c00 n:0x7ffeae2f9c04
}

//引用传递（直接操作原变量）
void ReferenceSwap(int& m, int& n){
  int temp = m;
  m = n;
  n = temp;
  cout << "m:" << m << " n:" << n << endl;    //m:15 n:10
}

void test(){
  int a = 10;
  int b = 15;
  //值传递
  // ValueSwap(a , b);
  // cout << "a:" << a << " b:" << b << endl; // a:10 b:15

  //地址传递
  // PointSwap(&a , &b);
  // cout << "a:" << a << " b:" << b << endl;  //  a:15 b:10

  //引用传递
  ReferenceSwap(a , b);
  cout << "a:" << a << " b:" << b << endl;    //  a:15 b:10
}
int main()
{
  test();
  return 0;
}

/*

引用的本质在c++内部实现是一个指针常量.
Type& ref = val; // Type* const ref = &val;



*/