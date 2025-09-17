#include<iostream>
using namespace std;

//  true(1值)，false(0值)
//  bool类型占1个字节大小

void test(){
  bool flag = false;
  cout << "false sizeof占位:"  <<sizeof(false)<< endl;
  cout << "false:"  <<false<< endl;

  flag = true;
  cout << "true sizeof占位:"  <<sizeof(flag)<< endl;
  cout << "true:"  <<flag<< endl;

  flag = 100;
  cout << "100转变为bool值:"  <<flag<< endl;
}
int main()
{
  test();
  return 0;
}
