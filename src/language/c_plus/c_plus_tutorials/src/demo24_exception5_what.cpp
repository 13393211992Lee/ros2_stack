#include<iostream>
using namespace std;
// m  ,  <>

class Students05 {
public:
	int m_age;
	Students05(int age) {
		if (age < 0 || age > 150) {
			//throw out_of_range("年龄在0到150之间");
			throw length_error("长度错误");
		}
		else {
			this->m_age = age;
		}
	}
};
int main()
{
  
  try {
		Students05 stu(152);
	}

  // 写法1：
	// catch (out_of_range &e) {
	// 	// 抛出异常的时候，写在out_of_range小括号中的内容，用e.what()出来
	// 	cout << e.what() << endl;
	// }

  	// 写法2：多态
	catch (exception& e) {  // exception是系统标准异常的基类，所以可以使用多态来实现
		// 抛出异常的时候，写在out_of_range小括号中的内容，用e.what()出来
		cout << e.what() << endl;
	}
  return 0;
}
