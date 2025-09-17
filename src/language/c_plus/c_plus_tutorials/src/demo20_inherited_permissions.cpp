#include<iostream>
using namespace std;

class Base2{
private:
  int private_a;
protected:
  int protected_b;
public:
  int public_c;
};

//公共继承
class Son1: public Base2{
public:
  void func(){
    // private_a = 100;
    protected_b = 200;
    public_c = 300;
  }
};

//保护继承
class Son2: protected Base2{
public:
  void func(){
    // private_a = 100;
    protected_b = 200;
    public_c = 300;
  }
};

//私有继承
class Son3: private Base2{
public:
  void func(){
    // private_a = 100;
    protected_b = 200;
    public_c = 300;
  }
};

class SonOfSon3: private Son3{
public:
  void func(){
    // public_a = 100;
    // protected_b = 200;
    // public_c = 300;
  }
};
int main()
{
//公开继承
  Son1 son1;
  // son1.private_a = 1;
  // son1.protected_b = 2;
  son1.public_c = 3;

// //保护继承
//   Son2 son2;
//   son2.private_a = 1;
//   son2.protected_b = 2;
//   son2.public_c = 3;
// //私有继承
//   Son3 son3;
//   son2.private_a = 1;
//   son2.protected_b = 2;
//   son2.public_c = 3;

  return 0;
}


/*






*/