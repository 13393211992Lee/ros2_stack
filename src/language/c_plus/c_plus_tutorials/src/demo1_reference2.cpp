#include<iostream>
using namespace std;
// m  ,  <>
// 引用作为函数参数
void intSwap(int& a , int& b){
  int temp =a;
  a=b;
  b=temp;
}

int& getMax(int& a,int& b){
  return a > b ? b : a;
}
int main()
{
  int x ,y;
  x = 10;
  y = 2;
  intSwap(x , y);
  cout << "x= "<< x << endl;
  cout << "y= "<< y << endl;
  
  cout << " getMax(x , y)= "<< getMax(x , y) << endl;

  
  return 0;
}



/*

引用常用于函数参数传递，以避免拷贝，并允许函数修改实参（如果不需要修改，则使用const引用）。


colcon build --packages-select c_tutorials
source install/setup.bash
ros2 run c_tutorials reference_2
*/