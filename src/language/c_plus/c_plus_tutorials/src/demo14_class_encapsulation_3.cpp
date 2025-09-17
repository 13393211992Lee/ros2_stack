#include<iostream>
using namespace std;

// 设计立方体类(Cube)，求出立方体的面积(2 * a * b + 2 * a * c + 2 * b * c)和体积(a* b* c)
// 分别用全局函数和成员函数判断两个立方体是否相等。

class Cube{
  private:
    int cube_l; // longth
    int cube_w; // width
    int cube_h; // height
  public:
    //getter
    int getCube_l(){
      return cube_l;
    }
    int getCube_w(){
      return cube_w;
    }
    int getCube_h(){
      return cube_h;
    }

    //setter
    void setCube_l(int clongth){
      cube_l = clongth;
    }
    void setCube_w(int cwidth){
      cube_w = cwidth;
    }
    void setCube_h(int cheight){
      cube_h = cheight;
    }

    // print
    void printTheClass(){
      cout << "cube_l: "<< cube_l << endl;
      cout << "cube_w: "<< cube_w << endl;
      cout << "cube_h: "<< cube_h << endl;
      
    }

    // s
    int cube_s(){
      return (cube_l*cube_w + cube_w*cube_h + cube_h*cube_l)*2;

    }

    // c
    int cube_c(){
      return (cube_l+cube_w+cube_h)*4;

    }

    //v
    int cube_v(){
      return cube_l*cube_w*cube_h;
    }
     	// 成员函数 实现判断是否相等
    bool com_cube_class(Cube &c2) {
      return cube_l == c2.getCube_l() && cube_h == c2.getCube_h() && cube_w == c2.getCube_w();
  }
};

//全局函数 实现判断是否相等
bool com_cube(Cube &c2,  Cube &c1){
  return c2.getCube_h() == c1.getCube_h() && c2.getCube_l() == c1.getCube_l() && c2.getCube_w() == c1.getCube_w();
}
int main()
{
  Cube cube;
  cube.setCube_l(10);
  cube.setCube_w(5);
  cube.setCube_h(1);
  cube.printTheClass();
  cout << "cube_c周长: "<< cube.cube_c() << endl;
  cout << "cube_s: "<< cube.cube_s() << endl;
  cout << "cube_v体积: "<< cube.cube_v() << endl;


  Cube cube2;
  cube2.setCube_l(10);
  cube2.setCube_w(5);
  cube2.setCube_h(1);
  cube2.printTheClass();
  cout << "cube_c周长: "<< cube2.cube_c() << endl;
  cout << "cube_s: "<< cube2.cube_s() << endl;
  cout << "cube_v体积: "<< cube2.cube_v() << endl;

  cout << "成员函数 实现判断是否相等" << endl;
  cout << "true1 or false0: "<< cube.com_cube_class(cube2) << endl;

  cout << "全局函数 实现判断是否相等" << endl;
  cout << "true1 or false0: "<< com_cube(cube , cube2) << endl;

  return 0;
}
