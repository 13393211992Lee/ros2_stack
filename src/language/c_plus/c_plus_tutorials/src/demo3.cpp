#include<iostream>
using namespace std;

/*命名空间只能全局范围内定义*/
namespace ns_A
{
  int a = 10;
  namespace ns_a{ int b = 13;}  //命名空间嵌套
} 

namespace ns_B
{
  int a = 20;
  void func(){
    cout << "Over and  Thanks" << endl;
  }
} 
//无名命名空间
namespace {
  int c = 30;
  void func_no_name(){
    cout << "无名命名空间" << endl;
  }
}

//命名空间别名
namespace veryLongName
{
  int d = 40;
  void func(){
    cout << "命名空间别名" << endl;
  }
} 

void test(){
  //定义和嵌套
	cout << "ns_A.a:" << ns_A::a << endl;
	cout << "ns_A.ns_a.b:" << ns_A::ns_a::b << endl;
	cout << "ns_B.a:" << ns_B::a << endl;
  ns_B::func();

  //无名命名空间
	cout << "c:" << c << endl;
  func_no_name();

  //命名空间别名
  namespace aShortName = veryLongName;
  cout << "aShortName.d: " << aShortName::d << endl;
  cout << "veryLongName.d: " << veryLongName::d << endl;
  aShortName::func();
  veryLongName::func();
  
}
int main(int argc, char const *argv[])
{
  (void)argc;
  (void)argv;
  test();
  return 0;
}
