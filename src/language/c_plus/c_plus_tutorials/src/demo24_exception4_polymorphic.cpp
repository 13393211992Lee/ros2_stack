#include<iostream>
using namespace std;
// m  ,  <>
// 异常的多态使用



// 异常基类
class BaseException{
public:
  virtual void printError()=0;

};

// 空指针的异常
class NullPointerException: public BaseException{
public:
  virtual void printError(){
    cout << "空指针异常" << endl;
  }
};

// 地址越界异常
class OutOfRangeException : public BaseException {
public:
	virtual void printError() {
		cout << "地址越界异常" << endl;
	}
};

void doThrow04() {
	//throw NullPointerException();
	throw OutOfRangeException();
}
int main()
{
  try
  {
    doThrow04();
  }
  catch(BaseException &e)
  {
    e.printError();
  }
  
  return EXIT_SUCCESS;
}
