#include<iostream>
#include<string.h>
using namespace std;

class Person
{
private:
public:
  char* pName;
  int mAge;
  Person();
  Person(const char* pName, int mAge);
  ~Person();
  void test();
  void ShowPerson();

};

Person::Person(){
  cout << "无参构造函数!" << endl;
  pName = (char*)malloc(strlen("undefined") + 1);
  strcpy(pName, "undefined");
  mAge = 0;
}
Person::Person(const char* name, int age ){
  cout << "有参构造函数!" << endl;
  pName = (char*)malloc(strlen(name) + 1);
  strcpy(pName, name);
  mAge = age;
}

void Person::test(){
	Person* person1 = new Person;
	Person* person2 = new Person("jah_con",33);

	person1->ShowPerson();
	person2->ShowPerson();

	delete person1;
	delete person2;
}
void Person::ShowPerson(){
		cout << "Name:" << pName << " Age:" << mAge << endl;
	}
Person::~Person(){
  cout << "析构函数!" << endl;
  if (pName != NULL){
    free(pName) ;
    pName = NULL;
  }
}


int main()
{ 
  Person p;
  p.test();

	return EXIT_SUCCESS;
  // return 0;
}
