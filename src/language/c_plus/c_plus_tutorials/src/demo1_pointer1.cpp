#include<iostream>
using namespace std;
// m  ,  <>
// 声明指针  获取地址 解引用（访问值） 指针运算
int main(void) 
{
  cout << "-------------------- " << '\n';
  int a = 10;
  cout << "a= " << a << endl;
  cout << "&a= " << &a << endl;        // `&a`：获取变量a的内存地址

  int* ptr = &a;
  cout << "ptr= " << ptr << endl;     
  cout << "&ptr = " << &ptr << endl;      
  cout << "*ptr = " << *ptr << endl;  //解引用操作（使用`*`）可以访问或修改指针所指向的内存地址中存储的值。

  *ptr = 20;
  cout << "a= " << a << endl;         // 解引用  修改 a 的值
  cout << "&a= " << &a << endl;     


  cout << "-------------------- " << '\n';
  int arr[3] = {50,40,30};
  int* ptr2 = arr;
  cout << "*ptr2= " << *ptr2 << '\n';
  ptr2++;
  cout << "*ptr2= " << *ptr2 << '\n';


  cout << "-------------------- " << '\n';
  string girl = "cici";
  string* ptr3 = &girl;
  cout << *ptr3 << '\n';
}

/*
1. 指针的基本概念
指针本质上是一个变量，其存储的是另一个变量的内存地址。
内存地址:计算机内存被划分为许多小的单元，每个单元都有一个唯一的地址（类似于门牌号）
指针变量:专门用来存储内存地址的变量。

`&a`：获取变量a的内存地址
int* ptr = &a;  ptr指针指向a的内存地址
*ptr2 ： 从ptr2中获取值
*/