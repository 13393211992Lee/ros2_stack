#include<iostream>
using namespace std;
// m  ,  <>

// 定义自己的异常类

class OwnException_OutofRange: public exception{
public:
  string e_msg;
  OwnException_OutofRange(const char* str){
    this->e_msg =  str;
  }
  OwnException_OutofRange(string str){
    this->e_msg =  str;
  }
  virtual char const* 
    what(){
    //  string 无法隐式转换为 const char* 需要手动转换
		return e_msg.c_str();
  }
};

class Students06 {
public:
	int m_age;
	Students06(int age) {
		if (age < 0 || age > 150) {
			//throw MyOutOfRange("年龄0到150");// const char*
			throw OwnException_OutofRange(string("年龄0到150"));//  string
      //const char* str
		}
		else {
			this->m_age = age;
		}
	}
};


int main()
{
  try
  {
    Students06 stu(1000);
  }
  catch(OwnException_OutofRange& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  return 0;
}

/*
如何编写自己的异常类？
① 建议自己的异常类要继承标准异常类。因为C++中可以抛出任何类型的异常，所以我们的异常类可以不继承自标准异常，
   但是这样可能会导致程序混乱，尤其是当我们多人协同开发时。
② 当继承标准异常类时，应该重载父类的what函数。
*/