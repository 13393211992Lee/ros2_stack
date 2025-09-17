//函数的默认参数

#include<iostream>
using namespace std;
int func1(int a, int b = 10, int c =8){
  // 传了实参，就用实参的。
	// 实参从左向右对应形参
	// 实参从左向右对应形参，如果出现默认值，右侧所有形参必须设置默认值
  return a+b+c;
}
// 函数的默认参数只能在声明或定义中设置一次
void func2(int a=5, int b=15);    //函数声明
//void func2(int a=5, int b=15){};  //函数定义（错误）
void func2(int a, int b){
  (void)a;
  (void)b;
};
int main()
{
  (void)func2; //`func2`转换为void类型，可能是为了抑制未使用函数的警告
  cout << "func1(10) : " << func1(10) << endl;
  cout << "func1(10,12,13) : " << func1(10,12,13) << endl;
  return EXIT_SUCCESS;
  //return 0;
}
/*
1函的默认参数从左向右，如果一个参数设置了默认参数，那么这个参数之后的参数都必须设置默认参数。
2如果函数声明和函数定义分开写，函数声明和函数定义不能同时设置默认参数。

1默认参数必须从右到左连续设置，也就是说，如果某个参数有默认值，那么它右边的所有参数都必须有默认值。
*/