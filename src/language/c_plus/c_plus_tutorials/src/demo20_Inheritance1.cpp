#include<iostream>
using namespace std;

class BaseContent
{
public:
  void header(){
    cout << "公共的头部" << endl;
  }
  void footer(){
    cout << "公共的底部" << endl;
  }
  void paging(){
    cout << "公共的分页" << endl;
  }
};

class Liuxing: public BaseContent
{
public:
  void content(){
    cout << "流行歌曲内容" << endl;
  }
};

class Huayu: public BaseContent{
public :
  void content(){
    cout << "华语歌曲内容" << endl;
  }
};

int main()
{
   Huayu huayu;
  cout << "---华语歌曲---" << endl;
  huayu.header();
  huayu.footer();
  huayu.paging();
  huayu.content();

  Liuxing liuxing;
  cout << "---流行歌曲---" << endl;
  liuxing.header();
  liuxing.footer();
  liuxing.paging();
  liuxing.content();
  return 0;
}

/*
派生类定义格式：
   Class 派生类名 :  继承方式 基类名{
         //派生类新增的数据成员和成员函数
   }
三种继承方式： 
        ◦ public ：    公有继承
        ◦ private ：   私有继承
        ◦ protected ： 保护继承
从继承源上分： 
    • 单继承：指每个派生类只直接继承了一个基类的特征
    • 多继承：指多个基类派生出一个派生类的继承关系,多继承的派生类直接继承了不止一个基类的特征
*/