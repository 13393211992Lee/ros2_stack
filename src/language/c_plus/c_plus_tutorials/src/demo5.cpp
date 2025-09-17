#include <cstdio>
#include <iostream>
using namespace std;
//C++函数检测增强
int sum(int a , int b){
  return a+b;
}
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  int s = sum(5 , 8);
  cout <<"sum : "<< s <<endl;
  return 0;
}
