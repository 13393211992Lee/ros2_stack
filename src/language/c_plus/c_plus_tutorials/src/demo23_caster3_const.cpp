#include<iostream>
using namespace std;
// m  ,  <>
//const_cast转换：不可以将非指针或非引用做const_cast转换

int main()
{

  const int* p = NULL;
	int* pp = const_cast<int*>(p);
	const int* ppp = const_cast<const int*>(pp);


  int num = 10;
	int& numRef = num;
	const int& num2 = const_cast<const int&>(numRef);
	int& num3 = const_cast<int&>(num2);

  // 不可以将非指针或非引用做const_cast转换
	// const int a = 10;
	// int a1 = const_cast<int>(a);

  // int a = 10;
	// const int a1 = const_cast<const int>(a);
  return 0;
}
