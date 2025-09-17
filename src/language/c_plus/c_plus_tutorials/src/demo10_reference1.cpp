#include<iostream>  //C++ 标准输入输出库std::cin |cout |cerr |clog
#include <cstdio>   //C 风格输入输出库 printf() scanf() fprintf() fscanf()
using namespace std;

void test1(){
  cout << " --------test1-------- "<< endl; 
  // 引用的语法：类型 &别名 = 原名
  int a = 10;
  int &b = a;
  cout << "b = " << b << endl;  //10

  //int &c;                     //引用必须初始化
}

void test2(){
  cout << " --------test2-------- "<< endl; 
  int a = 10;
  int &b = a;
  int c = 50;
  b = c;                      // 这里就不再是引用的意义，就是简单的赋值语句
  cout << "a = " << a << endl;  // 50
  cout << "b = " << b << endl;  // 50
  cout << "c = " << c << endl;  // 50
}

// 数组添加引用

void test3(){
  cout << " --------test3-------- "<< endl; 
  //1、直接建立引用
  int arr[10];
  int (&arr_b)[10] = arr;
  for (size_t i = 0; i < 10; i++)
  {
    arr[i] = i;
  }
  for (size_t i = 0; i < 10; i++)
  {
      cout << "arr_b[i] = " << arr_b[i] << endl; 

  }
  
  cout << " --------test4-------- "<< endl; 
  //2、先定义出数组的类型，再通过类型定义引用
  typedef int my_arr[10];           // 生成了一种有10个元素的数组类型
  my_arr &arr_c = arr;
  for (size_t i = 0; i < 10; i++)
  {
    cout << "arr_c[i] = " << arr_c[i] << endl; 

  }
  
}


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  //printf("hello world c_tutorials package\n");
  //cout << "hello world " << endl;
  test1();
  test2();
  test3();
  return 0;
}



/*
引用(reference)
    • 变量名实质上是一段连续内存空间的别名，是一个标号(门牌号)
    • 程序中通过变量来申请并命名内存空间
    • 通过变量的名字可以使用存储空间
基本语法: 
Type& ref = val;

注意事项：
    • &在此不是求地址运算，而是起标识作用。
    • 类型标识符是指目标变量的类型
    • 必须在声明引用变量时进行初始化。
    • 引用初始化之后不能改变。
    • 不能有NULL引用。必须确保引用是和一块合法的存储单元关联。
    • 可以建立对数组的引用。
*/