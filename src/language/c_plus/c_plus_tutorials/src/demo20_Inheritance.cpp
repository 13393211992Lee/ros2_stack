#include<iostream>
using namespace std;

class Huayu
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
  void content(){
    cout << "华语歌曲内容" << endl;
  }
};

class Liuxing
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
  void content(){
    cout << "流行歌曲内容" << endl;
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