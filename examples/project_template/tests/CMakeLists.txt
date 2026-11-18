cmake_minimum_required(VERSION 3.22)
project(MRLibCommandCenterExample LANGUAGES CXX)

# 独立主机工程，直接编译模板中的生产示例，不需要 Cube/HAL/RTOS。
enable_testing()
add_executable(cmd_center_example_tests
    cmd_center_host_tests.cpp
    ../app/cmd_center.cpp)
target_include_directories(cmd_center_example_tests PRIVATE .. ../../../User)
target_compile_features(cmd_center_example_tests PRIVATE cxx_std_17)
if(MSVC)
    target_compile_options(cmd_center_example_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(cmd_center_example_tests PRIVATE
        -Wall -Wextra -Wpedantic -Werror -fno-exceptions -fno-rtti -UNDEBUG)
endif()
add_test(NAME cmd_center_manual COMMAND cmd_center_example_tests)
add_test(NAME cmd_center_multi_source COMMAND cmd_center_example_tests multi)
add_test(NAME cmd_center_missing_selector COMMAND cmd_center_example_tests no-selector)
add_test(NAME cmd_center_custom_selector COMMAND cmd_center_example_tests custom-selector)
