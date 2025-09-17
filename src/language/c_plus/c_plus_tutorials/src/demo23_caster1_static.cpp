#include<iostream>
using namespace std;
// m  ,  <>

class Base01{};
class Son01: public Base01{};
class neighbor01{};

void test1(){
// 变量的类型转换static_cast<目标类型>(要转换的变量)
  char a = 'a';
  int int_a = static_cast<int>(a);
  cout<< int_a <<endl;
}

int main()
{
  test1();

  // 指针转换
	Base01* base = NULL;
	Son01* son = NULL;
	neighbor01* neighbor = NULL;

	// 将base父类转为son子类，向下类型转换，不安全
	Son01* son2 = static_cast<Son01*>(base);

  // 将son子类转为base父类，向上类型转换，安全
  Base01* base01 = static_cast<Base01*>(son);

  // 没有继承关系，指针转换失败
	// Base01* base3 = static_cast<Base01*>(neighbor);


  // 引用转换
	Base01 base1;
	Son01 son1;
	neighbor01 neighbor1;
  
  // 引用（起别名）
	Base01& base1_yin = base1;
	Son01& son1_yin = son1;
	neighbor01& neighbor1_yin = neighbor1;

	 
	// 父引用转子引用，不安全
	Son01& son2_yin = static_cast<Son01&>(base1_yin);

	// 子引用转父引用，安全
	Base01& base2_yin = static_cast<Base01&>(son1_yin);

	// 没有继承关系，引用转换失败
	//Base01& base2_yin = static_cast<Base01&>(neighbor1_yin);

  return 0;
}

/*
静态转换(static_cast)
    • 用于类层次结构中基类（父类）和派生类（子类）之间指针或引用的转换。
		• 进行上行转换（把派生类的指针或引用转换成基类表示）是安全的；
		• 进行下行转换（把基类指针或引用转换成派生类表示）时，由于没有动态类型检查，所以是不安全的。
    • 用于基本数据类型之间的转换，如把int转换成char，把char转换成int。这种转换的安全性也要开发人员来保证。
*/