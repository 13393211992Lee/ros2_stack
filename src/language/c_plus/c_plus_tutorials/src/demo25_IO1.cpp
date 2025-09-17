#include<iostream>
using namespace std;
// m  ,  <>
int main()
{
	// 2、cin.get(cha1) 带1个参数写法,实现链式编程	
	char char1, char2, char3, char4;
	cin.get(char1).get(char2).get(char3).get(char4);
	cout << char1 << " " << char2 << " " << char3 << " " << char4 << endl;
  return 0;
}
