#include<iostream>
namespace A
{
  int paramA = 20;
  int paramB = 21;
  void funcA(){
    std::cout<< "hello funcA" <<std::endl;
  }
  void funcB(){
    std::cout<< "hello funcB" <<std::endl;
  }
} 

namespace lol
{
  int hero_id = 1;
} 

namespace kun
{
  int hero_id = 3;
} 

//‘int lol::hero_id’ conflicts with a previous declaration
//hero_id 冲突
void test2(){
  //int hero_id = 2;
  //using lol::hero_id;
  //std::cout<< hero_id <<std::endl;
}

// 遵循就近原则，打印2
void test3(){
  int hero_id = 2;
  using namespace lol;
  std::cout << "test3 hero_id: " << hero_id <<std::endl;
}

//当using编译指令有多个，需要加前缀作用域作为区分
void test4(){
  using namespace lol;
  using namespace kun;
  std::cout<< "lol::hero_id : " << lol::hero_id << std::endl;
  std::cout<< "kun::hero_id : " << kun::hero_id << std::endl;
}
void test(){
  std::cout << "A.paramA:" << A::paramA << std::endl;
  A::funcA();

  //使用using
  using A::paramA;
  using A::funcA;
  using std::cout;
  using std::endl;
  cout << paramA << endl;
  funcA();
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  test4();
  
  return 0;
}
