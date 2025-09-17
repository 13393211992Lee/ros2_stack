#include<iostream>
using namespace std;
// m  ,  <>


class Base01 { virtual void fun() {} };
class Son01 : public Base01 { virtual void fun() {} };
class neighbor01 {};


int main()
{
  // 多态转换都是安全的
	// 父类指针指向子类的对象
	Base01* base = new Son01;

	// 将base父类转为son子类，向下类型转换，已经是多态了,所以安全
	Son01* son2 = dynamic_cast<Son01*>(base);




	// 父类引用指向子类对象
	Base01 base1;
	Son01 son1;

	// 引用（起别名）
	Base01& base1_yin = son1;

	// 父引用转子引用，已经是多态了,所以安全
	Son01& son2_yin = dynamic_cast<Son01&>(base1_yin);
	
  
  return 0;
}

/*
动态转换(dynamic_cast)
  • dynamic_cast主要用于类层次间的上行转换和下行转换；
  • 在类层次间进行上行转换时，dynamic_cast和static_cast的效果是一样的；
  • 在进行下行转换时，dynamic_cast具有类型检查的功能，比static_cast更安全；
*/