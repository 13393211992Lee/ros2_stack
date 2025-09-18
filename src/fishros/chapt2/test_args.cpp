#include<iostream>
/*
<<  m  ,  >>
~/ws_github_nav2/src/fishros/chapt2
# 编译
$ g++ ./test_args.cpp  
# 执行 
$ ./a.out --help        
$ ./a.out 
*/
int main(int argc, char* argv[]){
    std::cout<<"参数数量"<< argc <<std::endl;
    std::cout<<"第一个参数 程序name:"<< argv[0] <<std::endl;
    std::string arg1 = argv[1];
    if(arg1=="--help"){
        std::cout<< "程序帮助" << std::endl;
    }
}
