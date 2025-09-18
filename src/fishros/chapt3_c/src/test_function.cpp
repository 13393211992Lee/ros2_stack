#include <memory>
#include <iostream>
#include <functional>
void save_with_free_func(const std::string file_name){
    std::cout<<"自由函数"<< file_name <<std::endl;
}



class FileSave
{
public:
    void save_with_member_func(const std::string file_name){
        std::cout<<"成员函数"<< file_name <<std::endl;
    }  
private:

};


int main()
{
    FileSave file_save;
    // lambda函数
    auto save_with_lambda_func= [](const std::string &file_name) -> void {
        std::cout<<"lambda函数"<< file_name <<std::endl;
    };

    // 对应的调用方法
    save_with_free_func("file.txt");
    file_save.save_with_member_func("file.txt");
    save_with_lambda_func("file.txt");

    // 函数包装器
    std::function<void(const std::string&)> save_free = save_with_free_func;
    std::function<void(const std::string&)> save_lambda = save_with_lambda_func;
    //成员函数放入包装器
    std::function<void(const std::string&)> save_member = std::bind(
        &FileSave::save_with_member_func, &file_save ,std::placeholders::_1);
    
    //函数包装后的调用
    save_free("file_func.txt");
    save_lambda("file_func.txt");
    save_member("file_func.txt");

    return 0;
}
