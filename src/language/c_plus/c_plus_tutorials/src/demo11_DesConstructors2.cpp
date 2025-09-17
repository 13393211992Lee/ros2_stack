#include<iostream>
using namespace std;

class Phone
{
  private:
    /* data */
  public:
    string phone_name;
    Phone(string p_name);
    ~Phone();
  };

  class App
  {
  private:
    /* data */
  public:
    string app_name;
    App(string a_name);
    ~App();
};

Phone::Phone(string p_name)
{
  phone_name = p_name;
  cout << " phone的构造函数调用 "<< endl;
}

Phone::~Phone()
{
  cout << " phone的析构函数调用 "<< endl;
}

App::App(string a_name)
{
  app_name = a_name;
  cout << " App的构造函数调用 "<< endl;
}

App::~App()
{
  cout << " App的析构函数调用 "<< endl;
}



class Student
{
  private:
    /* data */
  public:
    string stu_name;
    Phone stu_phone;
    App stu_app;
    Student(string stu_name, string p_name , string a_name);
    ~Student();
    void usePhone();
};
void Student::usePhone(){
  cout << stu_name << "拿着" << stu_phone.phone_name << "手机，刷着" << stu_app.app_name << endl;
}

Student::Student(string stu_name, string p_name , string a_name):
      stu_name(stu_name),stu_phone(p_name),stu_app(a_name){       
  cout << " Student的构造函数调用 "<< endl;
}

Student::~Student()
{
  cout << " Student的析构函数调用 "<< endl;
}



int main()
{
  // Phone p("apple");
  // App app("tender");
  Student stu(" cici", "apple", "wechat");
  stu.usePhone();
  return 0;
}


/*

在类中定义的数据成员一般都是基本的数据类型。但是类中的成员也可以是对象，叫做对象成员。
C++中对对象的初始化是非常重要的操作，当创建一个对象的时候，
c++编译器必须确保调用了所有子对象的构造函数。如果所有的子对象有默认构造函数，
编译器可以自动调用他们。先调用对象成员的构造函数，再调用本身的构造函数。
析构函数和构造函数调用顺序相反，先构造，后析构。


$ ros2 run c_tutorials c_d11_2
 phone的构造函数调用 
 App的构造函数调用 
 Student的构造函数调用 
 cici拿着apple手机，刷着wechat
 Student的析构函数调用 
 App的析构函数调用 
 phone的析构函数调用 

*/