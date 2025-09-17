#include<iostream>
using namespace std;
// m  ,  <>
int main()
{
  // 1、cin.get() 等待屏幕输入内容一个字符，保存到c变量中
	// 输入ab回车：  第一次：a  第二次：b   第三次：回车  第四次：等待下个输入
	// 回车会遗留在缓冲区中
  char c = cin.get();
	cout << "c:" << c << endl;

	c = cin.get();
	cout << "c:" << c << endl;

	c = cin.get();
	cout << "c:" << c << endl;

	c = cin.get();
	cout << "c:" << c << endl;
  return 0;
}
