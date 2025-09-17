#include<iostream>
using namespace std;  
class MyString
{
private:
  /* data */
public:
  explicit MyString(int n);   //构造函数被声明为explicit，这意味着这个构造函数不能用于隐式转换
  MyString(const char* str);
  ~MyString();
};

MyString::MyString(int n){
  cout << "MyString(int n)" << n << endl;
}
MyString::MyString(const char* str){
  cout << "MyString(const char* str)"<< str << endl;
}
MyString::~MyString()
{
}

int main()
{
  MyString str(1);
  // MyString str2 = 2;   //赋值形式的初始化 因为要隐式转化 所以explicit

  MyString str3("abcd");
  MyString str4 = "abcd";

  return EXIT_SUCCESS;
}

/*
explicit，禁止通过构造函数进行的隐式转换。

第一个构造函数被声明为explicit，这意味着这个构造函数不能用于隐式转换。
也就是说，当尝试用赋值操作符或者隐式转换来初始化对象时，explicit构造函数不会被自动调用

// MyString str2 = 2;
因为这里的初始化使用的是赋值形式的初始化，这通常会尝试将右边的类型转换为MyString类型。
但是因为构造函数是explicit的，所以不能进行隐式转换

explicit总结：
  赋值形式的初始化 不可使用； eg: MyString str2 = 2
  直接调用构造函数 可以只用； eg:MyString str(1);
*/