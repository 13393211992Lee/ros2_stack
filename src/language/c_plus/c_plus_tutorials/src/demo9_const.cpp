#include<iostream>  //C++ 标准输入输出库std::cin |cout |cerr |clog
#include <cstdio>   //C 风格输入输出库 printf() scanf() fprintf() fscanf()
using namespace std;

void test(){
  const int  _VALUE = 100;
  //_VALUE = 200;                       //const 用于声明一个值不可变的变量（常量）

}
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  printf("hello world c_tutorials package\n");
  cout << "hello world " << endl;
  return 0;
}


/*
尽量以const替换#define
  1． const有类型，可进行编译器类型安全检查。#define无类型，不可进行类型检查.
  2． const有作用域，而#define不重视作用域，默认定义处到文件结尾.如果定义在指定作用域下有效的常量，那么#define就不能用。
*/