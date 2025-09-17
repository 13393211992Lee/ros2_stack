cmake_minimum_required(VERSION 3.8)
project(c_plus_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(c_d1 src/demo1.cpp)
add_executable(c_d2 src/demo2.cpp)
add_executable(c_d3 src/demo3.cpp)
add_executable(c_d4 src/demo4_using.cpp)
add_executable(c_d5 src/demo5.cpp)
add_executable(c_d6 src/demo6_class.cpp)
add_executable(c_d7 src/demo7_bool.cpp)
add_executable(c_d8 src/demo8_trinocular.cpp)
add_executable(c_d9b 
  src/demo9_const_a.cpp #被引用的文件也要写进去
  src/demo9_const_a_run.cpp
)
add_executable(c_d10 src/demo10_reference1.cpp)
add_executable(c_d10_2 src/demo10_reference2.cpp)
#add_executable(c_d10_3 src/demo10_reference3.cpp)
add_executable(c_d10_4 src/demo10_reference4.cpp)
add_executable(c_d12 src/demo12_func_default_param_value.cpp)
add_executable(c_d13 src/demo13_func_overload.cpp)
add_executable(c_d14_1 src/demo14_class_encapsulation_1.cpp)
add_executable(c_d14_2 src/demo14_class_encapsulation_2.cpp)
add_executable(c_d14_3 src/demo14_class_encapsulation_3.cpp)
add_executable(c_d11 src/demo11_DeepShallowCopy.cpp)
add_executable(c_d11_1 src/demo11_DesConstructors.cpp)
add_executable(c_d11_2 src/demo11_DesConstructors2.cpp)
add_executable(c_d11_3 src/demo11_DesConstructors3_explicit.cpp)
add_executable(c_d15_3 src/demo15_Dynamic_object_delete.cpp)
add_executable(c_d16 src/demo16_newdel_array.cpp)
add_executable(c_d17 src/demo17_static_var.cpp)
add_executable(c_d17_2 src/demo17_static_func.cpp)
add_executable(c_d17_3 src/demo17_static_var_singleton_mode.cpp)
add_executable(c_d18 src/demo18.cpp)
add_executable(c_d18_1 src/demo18_this.cpp)
add_executable(c_d18_2 src/demo18_const_func.cpp)
add_executable(c_d18_3 src/demo18_const_object.cpp)
add_executable(c_d20 src/demo20_Inheritance.cpp)
add_executable(c_d20_1 src/demo20_Inheritance1.cpp)
add_executable(c_d20_2 src/demo20_inherited_obj.cpp)
add_executable(c_d20_3 src/demo20_inheritance_cons_des_tructor.cpp)
add_executable(c_d20_4 src/demo20_inherited_same_problem.cpp)
add_executable(c_d20_5 src/demo20_inherited_static_obj.cpp)
add_executable(c_d20_6 src/demo20_inheritance_more.cpp)
add_executable(c_d21 src/demo21_polymorphism.cpp)
add_executable(c_d21_2 src/demo21_polymorphism2.cpp)
add_executable(c_d21_3 src/demo21_polymorphism3_pure_virtual_func.cpp)
add_executable(c_d22_1 src/demo22_template_func.cpp)
add_executable(c_d22_2 src/demo22_template2_to_sort.cpp)
add_executable(c_d22_3 src/demo22_templatefunc_and_func.cpp)
add_executable(c_d22_4 src/demo22_templatefunc_regularfunc_callrules.cpp)
add_executable(c_d22_5 src/demo22_template_func_limitation.cpp)
add_executable(c_d22_6 src/demo22_template_class.cpp)
add_executable(c_d22_7 src/demo22_template7_class_tobe_func_param.cpp)
add_executable(c_d22_8 src/demo22_template8_class_derive_ordinary_class.cpp)
add_executable(c_d22_9 
  src/demo22_9main.cpp
  src/demo22_9.hpp)  #被引用的文件也要写进去
add_executable(c_d22_10 src/demo22_template10_class_and_friendlyclass.cpp)
add_executable(c_d22_11 src/demo22_template11_class_and_friendlyclass.cpp)

add_executable(caster23_1 src/demo23_caster1_static.cpp)
add_executable(exception24 src/demo24_exception.cpp)
add_executable(exception24_1 src/demo24_exception1.cpp)
add_executable(exception24_2 src/demo24_exception2_unwinding.cpp)
add_executable(exception24_3_1 src/demo24_exception3_lifecycle1.cpp)
add_executable(exception24_3_2 src/demo24_exception3_lifecycle2.cpp)
add_executable(exception24_3_3 src/demo24_exception3_lifecycle3.cpp)
add_executable(exception24_4 src/demo24_exception4_polymorphic.cpp)
add_executable(exception24_5 src/demo24_exception5_what.cpp)
add_executable(exception24_6 src/demo24_exception6_define_owner.cpp)
add_executable(IO_25_1 src/demo25_IO1.cpp)
add_executable(IO_25_2 src/demo25_IO2.cpp)
add_executable(IO_25_3 src/demo25_IO3.cpp)
add_executable(pointer_1 src/demo1_pointer1.cpp)
add_executable(pointer_2 src/demo1_pointer2.cpp)
add_executable(reference_1 src/demo1_reference1.cpp)
add_executable(reference_2 src/demo1_reference2.cpp)

install(TARGETS 
c_d1
c_d2
c_d3
c_d4
c_d5
c_d6
c_d7
c_d8
c_d9b
c_d10
c_d10_2
#c_d10_3
c_d10_4
c_d12
c_d13
c_d14_1
c_d14_2
c_d14_3
c_d11
c_d11_1
c_d11_2
c_d11_3
c_d15_3
c_d16
c_d17
c_d17_2
c_d17_3
c_d18
c_d18_1
c_d18_2
c_d18_3
c_d20
c_d20_1
c_d20_2
c_d20_3
c_d20_4
c_d20_5
c_d20_6
c_d21
c_d21_2
c_d21_3
c_d22_1
c_d22_2
c_d22_3
c_d22_4
c_d22_5
c_d22_6
c_d22_7
c_d22_8
c_d22_9
c_d22_10
c_d22_11
caster23_1
exception24
exception24_1
exception24_2
exception24_3_1
exception24_3_2
exception24_3_3
exception24_4
exception24_5
exception24_6
IO_25_1
IO_25_2
IO_25_3
pointer_1
pointer_2
reference_1
reference_2
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
