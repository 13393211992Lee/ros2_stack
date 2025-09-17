#include <cstdio>
#include <iostream>
using namespace std;


class Student {
private:
    int id;
    std::string name;
    float score;

public:
    Student(int i, const std::string& n, float s) 
        : id(i), name(n), score(s) {}

    // 提供受控的访问方法
    void setScore(float s) {
        if (s >= 0 && s <= 100) score = s;
    }

    // 打印学生信息
    void printInfo() const {
        std::cout << "ID: " << id 
                  << ", Name: " << name 
                  << ", Score: " << score 
                  << std::endl;
    }
};

int main(int argc, char const *argv[])
{
  (void)argc;
  (void)argv;
  // 实例化方式 1：直接初始化
  Student stu1(10,"cici",98.3);
  stu1.printInfo();

  // 实例化方式 2：动态分配（堆内存）
  Student* stu2 = new Student(11,"cici2",93.3);
  stu2->printInfo();

  // 赋值操作
  stu1.setScore(95.0);  // 通过方法修改值
  stu1.printInfo();

  // 释放堆内存
  delete stu2;
  return 0;
}
