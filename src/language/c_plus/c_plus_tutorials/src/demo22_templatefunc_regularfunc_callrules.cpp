#include<iostream>
using namespace std;
// m  ,  <>
//函数模板和普通函数在一起调用规则

template<typename T>
void myPrint(T a ,T b){
  cout << "template function called...(myPrint(T a ,T b)) " << endl;
}

template<typename T>
void myPrint(T a ,T b ,T c){
  cout << "template function 3c called... (myPrint(T a ,T b ,T c))" << endl;
}

void myPrint(int a , int b){
  cout << "regular function called...(myPrint(int a , int b)) " << endl;
}
int main()
{
  int a = 1;
	int b = 2;
	int c = 3;
	// 1、函数模板和普通函数都可以调佣的时候，优先调用普通函数
	myPrint(a,b);
  // 2、如果就想调用函数模板，可以使用空模板参数列表
	myPrint<>(a, b);
  // 3、函数模板可以发生函数重载
	myPrint(a,b,c);

  // 4、如果模板可以产生更好的匹配，就会优先执行函数模板
	char d = 'd';
	char f = 'f';
	myPrint(d, f);
  return 0;
}

/* 函数模板和普通函数在一起调用规则
    • c++编译器优先考虑普通函数
    • 可以通过空模板实参列表的语法限定编译器只能通过模板匹配
    • 函数模板可以像普通函数那样可以被重载
    • 如果函数模板可以产生一个更好的匹配，那么选择模板
*/
