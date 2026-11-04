cmake_minimum_required(VERSION 3.22)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(MR_TestLib LANGUAGES C CXX)
    set(MR_TESTLIB_HOST_TESTS_DEFAULT ON)
else()
    set(MR_TESTLIB_HOST_TESTS_DEFAULT OFF)
endif()

option(MR_TESTLIB_BUILD_HOST_TESTS "Build host-side regression tests"
       ${MR_TESTLIB_HOST_TESTS_DEFAULT})
option(MR_TESTLIB_BUILD_ARM_HOST_TESTS "Build arm host tests" ON)
option(MR_TESTLIB_BUILD_CHASSIS_HOST_TESTS "Build chassis host tests" ON)

add_library(mr_component_math INTERFACE)
add_library(mr::component_math ALIAS mr_component_math)
target_compile_features(mr_component_math INTERFACE cxx_std_17)
target_include_directories(mr_component_math INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/User
)

add_library(mr_component_filter INTERFACE)
add_library(mr::component_filter ALIAS mr_component_filter)
target_link_libraries(mr_component_filter INTERFACE mr_component_math)
target_include_directories(mr_component_filter INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/User
)

add_library(mr_component_controller INTERFACE)
add_library(mr::component_controller ALIAS mr_component_controller)
target_link_libraries(mr_component_controller INTERFACE
    mr_component_filter
    mr_component_math
)
target_include_directories(mr_component_controller INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/User
)

add_library(mr_robotics_chassis INTERFACE)
add_library(mr::robotics_chassis ALIAS mr_robotics_chassis)
target_link_libraries(mr_robotics_chassis INTERFACE mr_component_math)
target_compile_features(mr_robotics_chassis INTERFACE cxx_std_17)
target_include_directories(mr_robotics_chassis INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/User
)

add_library(mr_device_wheel_interface INTERFACE)
add_library(mr::device_wheel_interface ALIAS mr_device_wheel_interface)
target_compile_features(mr_device_wheel_interface INTERFACE cxx_std_17)
target_include_directories(mr_device_wheel_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/User
)

add_library(mr_module_chassis INTERFACE)
add_library(mr::module_chassis ALIAS mr_module_chassis)
target_link_libraries(mr_module_chassis INTERFACE
    mr_robotics_chassis
    mr_component_controller
    mr_device_wheel_interface
)
target_include_directories(mr_module_chassis INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/User
)
target_compile_features(mr_module_chassis INTERFACE cxx_std_17)

add_library(mr_robotics_arm INTERFACE)
add_library(mr::robotics_arm ALIAS mr_robotics_arm)
target_link_libraries(mr_robotics_arm INTERFACE mr_component_math)
target_compile_features(mr_robotics_arm INTERFACE cxx_std_17)
target_include_directories(mr_robotics_arm INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/User
)
target_sources(mr_robotics_arm INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/User/robotics/core/rotation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User/robotics/core/transform.cpp
)

if(MR_TESTLIB_BUILD_HOST_TESTS)
    enable_testing()

    if(MR_TESTLIB_BUILD_ARM_HOST_TESTS)
        add_executable(mr_arm_host_tests
            User/robotics/arm/tests/arm_host_tests.cpp
        )
        target_link_libraries(mr_arm_host_tests PRIVATE mr_robotics_arm)
        target_compile_options(mr_arm_host_tests PRIVATE
            -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti
        )
        add_test(NAME mr_arm_host_tests COMMAND mr_arm_host_tests)
    endif()

    if(MR_TESTLIB_BUILD_CHASSIS_HOST_TESTS)
        add_executable(mr_chassis_host_tests
            User/robotics/chassis/tests/chassis_host_tests.cpp
        )
        target_link_libraries(mr_chassis_host_tests PRIVATE mr_robotics_chassis)
        target_compile_options(mr_chassis_host_tests PRIVATE
            -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti
        )
        add_test(NAME mr_chassis_host_tests COMMAND mr_chassis_host_tests)

        add_executable(mr_chassis_module_host_tests
            User/module/chassis/tests/chassis_module_host_tests.cpp
        )
        target_link_libraries(mr_chassis_module_host_tests PRIVATE
            mr_module_chassis
        )
        target_compile_options(mr_chassis_module_host_tests PRIVATE
            -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti
        )
        add_test(NAME mr_chassis_module_host_tests
                 COMMAND mr_chassis_module_host_tests)
    endif()
endif()
