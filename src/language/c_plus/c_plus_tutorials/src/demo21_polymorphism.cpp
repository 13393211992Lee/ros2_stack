#include<iostream>
using namespace std;

class Animal10 {
public:
	virtual void speak() {   // 虚函数
		cout << "动物在叫" << endl;
	}
};
class Cat : public Animal10 {
public:
	virtual void speak() {  // 子类中的virtual可以省略不写
		cout << "喵喵喵" << endl;
	}
};

class Dog : public Animal10 {
public:
	void speak() {
		cout << "汪汪汪" << endl;
	}
};

//父类的引用指向子类的对象
void speaking(Animal10 &animal) { // Animal10 &animal = cat1;
	// 地址早绑定好了，属于静态联编
	
	// 如果想让小猫咪叫，就需要后绑定。需要在运行阶段再去绑定函数的地址，叫动态联编
	animal.speak();
}

int main()
{
  Cat cat1;
	speaking(cat1);

	Dog dog1;
	speaking(dog1);


	// 父类的指针指向子类的对象
	Animal10* animal = new Cat;
	animal->speak();

	Animal10* animal2 = new Dog;
	animal2->speak();
	return EXIT_SUCCESS;
}
