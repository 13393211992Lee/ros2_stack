#include<iostream>
using namespace std;
// m  ,  <>
int increase(int* a){
  return *a += 10;
}
void test(){
  int a = 5;
  int* p1 = &a;     // p1指向a
  int** p2 = &p1;   // p2指向p1（二级指针）
  cout << a << endl;    // 5
  cout << *p1 << endl;  // 5
  cout << **p2 << endl; // 5
  **p2 = 10;            // 修改a的值
  cout << a << endl;    // 10
}
int main()
{
  int b =9;
  cout<< increase(&b)<<endl;
  test();
  return 0;
}


/*
colcon build --packages-select c_tutorials
source install/setup.bash
ros2 run c_tutorials pointer_2
*/