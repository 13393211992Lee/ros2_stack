#include<iostream>
using namespace std;
// m  ,  <>
int main()
{
  cout << "-------reference & pointer--------  "<< endl; 
  int a = 10;
  cout << "a=  " << a << endl;          // a=  10
  cout << "&a=  " << &a << endl;        //  &a=  0x7fff9a8aa200

  int& ref = a;
  cout << "ref=  " << ref << endl;      // ref=  10
  cout << "&ref=  " << &ref << endl;    // &ref=  0x7fff9a8aa200
  
  int* ptr = &a;
  cout << "ptr=  " << ptr << endl;      // ptr=  0x7fff9a8aa200
  cout << "&ptr=  " << &ptr << endl;    // &ptr=  0x7fff9a8aa208(指针地址)
  cout << "*ptr=  " << *ptr << endl;    // *ptr=  10  (解指针)

  int y = 30;
  ref =  y;             // 2. 不可重新绑定   y的值 赋值赋值给 a的引用ref. 
  cout << "a=  " << a << endl;      // 30


    
  // 常引用（const引用）：
  // 可以绑定到常量或临时对象，并且不能通过常引用修改所引用的对象。
  const int& ref2 = 10;                 //常引用可以绑定到字面值

  cout << "-------const引用--------  "<< endl;  
  int a2 = 5;
  const int& ref3 = a2;
  cout << "ref3=  " << ref3 << endl;
  // ref3 = 55; 不能通过cref2修改a
  a2 = 55;
  cout << "a2=  " << a2 << endl;
  return 0;
}

/*

colcon build --packages-select c_tutorials
source install/setup.bash
ros2 run c_tutorials reference_1
*/

/*
引用的特点
  1. 引用在创建时必须初始化。
  2. 不可重新绑定 (引用一旦绑定到一个对象，就不能再绑定到另一个对象)
  3. 引用必须指向有效的对象，不存在空引用
  4. 与指针的区别 :引用更安全（无空引用），但指针可以重新指向和为空。
*/