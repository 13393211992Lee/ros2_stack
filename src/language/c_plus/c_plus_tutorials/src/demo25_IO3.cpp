#include<iostream>
using namespace std;
// m  ,  <>
int main()
{
// 3、cin.get(字符数组,读取个数) :  读取字符串到字符数组中
	char buf[1000] = { 0 };  // 字符数组
	cin.get(buf,1000);
	
	char c = cin.get();
	if (c == '\n') {
		cout << "换行符遗留在缓冲区中" << endl;
	}
	else {
		cout << "换行符没有遗留在缓冲区中" << endl;
	}
	cout << buf << endl;
}
