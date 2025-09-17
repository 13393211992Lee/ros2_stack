#include<iostream>
#include <string.h>
using namespace std;


// C动态分配内存方法


class Person{
  private:

  public:
    int mAge;
    char* pName;
    Person();
    ~Person();
    void Init();
    void Clean();
};

Person::Person(){
  mAge = 20;
  pName = (char*)malloc(strlen("john")+1);
  strcpy(pName, "john");
}
void Person::Init(){
  mAge = 20;
  pName = (char*)malloc(strlen("john")+1);
  strcpy(pName, "john");
}

void Person::Clean(){
  if (pName != NULL){
    free(pName);
  }
}
Person::~Person(){
}

int main()
{
  Person* person = (Person*)malloc(sizeof(Person));
  if(person == NULL){
		return 0;
	}
  //调用初始化函数
	person->Init();
	//清理对象
	person->Clean();
	//释放person对象
	free(person);

	return EXIT_SUCCESS;
  // return 0;
}
