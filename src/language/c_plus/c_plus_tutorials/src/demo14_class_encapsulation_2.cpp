#include<iostream>
using namespace std;
class Person
{
private:
  string p_name;
  int p_age;
  int p_pwd;
public:
  void setName(string name){  //设置名字
    p_name = name;
  }
  string getName(){           // 获取名字
    return p_name;
  }
  int getAge(){
    return p_age;
  }
  void setAge(int age){
    p_age = age;
  }
  Person(/* args */);
  ~Person();
};

Person::Person(/* args */)
{
}

Person::~Person()
{
}

int main()
{
  class Person p8;
	// cout << "name: "<<p8.p_name << endl;   // 私有属性，无法外部获取
	// cout << "age: "<<p8.p_age << endl;
	// cout << "pwd: "<<p8.p_pwd << endl;

  p8.setAge(10);
  p8.setName("cici");
	cout << "p8.name: "<<p8.getName() << endl;
	cout << "p8.age: "<<p8.getAge() << endl;
  
  return 0;
}
