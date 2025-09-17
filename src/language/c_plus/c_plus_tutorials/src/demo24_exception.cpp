#include<iostream>
using namespace std;
// m  ,  <>
class MyException{
public:
  void printE(){
    cout << "手写异常" <<endl;
  }
};

class Student{
  public:
    Student(){
      cout << "默认构造" << endl;
    }
    ~Student(){
      cout << "析构函数" << endl;
    }
};

int cal(int a, int b) {
  if (b == 0) {
    Student stu1;
    Student stu2;
    throw MyException(); 
  }else{
    return a / b;
  }
}

void test2(){
  int a = 10;
	int b = 0;
  try{
    cal(a ,b);
  }catch(int){
    cout << "int类型的捕获异常" << endl;
  }catch(char){
    cout << "char类型的捕获异常" << endl;
  }catch (double) { 
		cout << "double类型的捕获异常" << endl;
	}catch (MyException e) {   
		e.printE();
	}catch (...) {   // 除了以上几种类型之外，都在这里捕获异常
		throw; // 我这里不处理，让后续异常处理程序处理
		cout << "其它类型的捕获异常" << endl;
	}
  
}
int main()
{
  // 异常的嵌套，异常的传递如何实现：从内向外传递。务必要有函数处理，否则会报错
	try {
		test2();
	}
	catch (char) { // 捕获异常后的代码
		cout << "main中的char类型的捕获异常" << endl;
	}
	catch (...) {   // 除了以上几种类型之外，都在这里捕获异常
		cout << "main中的其它类型的捕获异常" << endl;
	}
	return EXIT_SUCCESS;
}

/*
总结:
  • 若有异常则通过throw操作创建一个异常对象并抛出。
  • 将可能抛出异常的程序段放到try块之中。
  • 如果在try段执行期间没有引起异常，那么跟在try后面的catch子句就不会执行。
  • catch子句会根据出现的先后顺序被检查，匹配的catch语句捕获并处理异常(或继续抛出异常)
  • 如果匹配的处理未找到，则运行函数terminate将自动被调用，其缺省功能调用abort终止程序。
  • 处理不了的异常，可以在catch的最后一个分支，使用throw，向上抛。

  c++异常处理使得异常的引发和异常的处理不必在一个函数中，这样底层的函数可以着重解决具体问题，
  而不必过多的考虑异常的处理。上层调用者可以在适当的位置设计对不同类型异常的处理。
*/