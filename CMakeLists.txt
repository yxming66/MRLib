cmake_minimum_required(VERSION 3.22)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(MR_TestLib LANGUAGES C CXX)
    set(MR_TESTLIB_HOST_TESTS_DEFAULT ON)
else()
    # CMake must enable languages in the common parent of consumers and library.
    if(NOT CMAKE_C_COMPILER_LOADED OR NOT CMAKE_CXX_COMPILER_LOADED)
        message(FATAL_ERROR "MR_TestLib requires project(... LANGUAGES C CXX) before add_subdirectory")
    endif()
    set(MR_TESTLIB_HOST_TESTS_DEFAULT OFF)
endif()

option(MR_TESTLIB_BUILD_HOST_TESTS "Build host-side regression tests"
       ${MR_TESTLIB_HOST_TESTS_DEFAULT})
option(MR_TESTLIB_BUILD_ARM_HOST_TESTS "Build arm host tests" ON)
option(MR_TESTLIB_BUILD_CHASSIS_HOST_TESTS "Build chassis host tests" ON)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/MrTargets.cmake")

if(MR_TESTLIB_BUILD_HOST_TESTS)
    enable_testing()

    add_executable(mr_manual_assembly_host_tests tests/integration/manual_assembly_host_tests.cpp)
    target_include_directories(mr_manual_assembly_host_tests PRIVATE "${CMAKE_CURRENT_LIST_DIR}/tests")
    target_link_libraries(mr_manual_assembly_host_tests PRIVATE
        mr::module_arm mr::module_chassis mr::device_motor mr::device_wheel mr::device_joint)
    target_compile_options(mr_manual_assembly_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti -UNDEBUG)
    add_test(NAME mr_manual_assembly_host_tests COMMAND mr_manual_assembly_host_tests)

    add_executable(mr_periodic_task_host_tests tests/task/periodic_task_host_tests.cpp)
    target_include_directories(mr_periodic_task_host_tests BEFORE PRIVATE tests/stubs/task User)
    target_compile_features(mr_periodic_task_host_tests PRIVATE cxx_std_17)
    # The host CMSIS stub throws only to leave an infinite task in the test.
    target_compile_options(mr_periodic_task_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -UNDEBUG)
    add_test(NAME mr_periodic_task_host_tests COMMAND mr_periodic_task_host_tests)

    add_executable(mr_execution_host_tests tests/device/execution_host_tests.cpp)
    target_link_libraries(mr_execution_host_tests PRIVATE mr_device_joint mr_device_wheel)
    add_executable(mr_controller_learning_host_tests tests/device/controller_learning_host_tests.cpp)
    target_include_directories(mr_controller_learning_host_tests PRIVATE "${CMAKE_CURRENT_LIST_DIR}/tests")
    target_link_libraries(mr_controller_learning_host_tests PRIVATE
        mr_device_motor_controller mr_device_soft_limit_learning)
    add_executable(mr_motor_units_host_tests tests/device/motor_units_host_tests.cpp)
    target_include_directories(mr_motor_units_host_tests PRIVATE "${CMAKE_CURRENT_LIST_DIR}/tests"
        "${CMAKE_CURRENT_LIST_DIR}/examples/device_execution/app")
    target_link_libraries(mr_motor_units_host_tests PRIVATE mr_device_motor mr_device_wheel mr_module_chassis)
    foreach(_mr_test IN ITEMS mr_execution_host_tests mr_controller_learning_host_tests mr_motor_units_host_tests)
        set_target_properties(${_mr_test} PROPERTIES
            CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
        target_compile_options(${_mr_test} PRIVATE
            -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti -UNDEBUG)
        add_test(NAME ${_mr_test} COMMAND ${_mr_test})
    endforeach()

    # Compile the controller package against the host BSP stub to keep the
    # board clock injectable and avoid accidental timer dependencies on BSP.
    add_library(mr_motor_controller_compile_check OBJECT
        User/device/motor/packages/controller/motor_controller.cpp)
    target_include_directories(mr_motor_controller_compile_check BEFORE PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/tests/stubs")
    target_include_directories(mr_motor_controller_compile_check PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/User")
    set_target_properties(mr_motor_controller_compile_check PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_motor_controller_compile_check PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -Wno-comment
        -fno-exceptions -fno-rtti)

    add_executable(mr_motor_controller_config_host_tests
        tests/device/motor_controller_config_host_tests.cpp)
    target_include_directories(mr_motor_controller_config_host_tests BEFORE PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/tests/stubs")
    target_include_directories(mr_motor_controller_config_host_tests PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/User")
    set_target_properties(mr_motor_controller_config_host_tests PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_motor_controller_config_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -Wno-comment
        -fno-exceptions -fno-rtti)
    add_test(NAME mr_motor_controller_config_host_tests
             COMMAND mr_motor_controller_config_host_tests)

    add_executable(mr_joint_host_tests tests/device/joint_host_tests.cpp)
    target_include_directories(mr_joint_host_tests PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/User")
    set_target_properties(mr_joint_host_tests PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_joint_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti)
    add_test(NAME mr_joint_host_tests COMMAND mr_joint_host_tests)

    add_library(mr_joint_adapter_compile_check OBJECT
        tests/device/joint_motor_adapter_compile_check.cpp)
    target_include_directories(mr_joint_adapter_compile_check BEFORE PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/tests/stubs")
    target_include_directories(mr_joint_adapter_compile_check PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/User")
    set_target_properties(mr_joint_adapter_compile_check PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_joint_adapter_compile_check PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -Wno-comment
        -fno-exceptions -fno-rtti)

    add_executable(mr_arm_module_host_tests
        tests/module/arm_module_host_tests.cpp
        examples/three_axis_arm/app/arm_model_config.cpp
        examples/three_axis_arm/app/arm_command_adapter.cpp)
    target_include_directories(mr_arm_module_host_tests PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/examples/three_axis_arm/app")
    target_link_libraries(mr_arm_module_host_tests PRIVATE mr_module_arm)
    set_target_properties(mr_arm_module_host_tests PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_arm_module_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors
        -fno-exceptions -fno-rtti)
    add_test(NAME mr_arm_module_host_tests COMMAND mr_arm_module_host_tests)

    add_executable(mr_music_host_tests tests/module/music_host_tests.c
        resources/music/nokia.c resources/music/rm.c)
    target_include_directories(mr_music_host_tests PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/resources/music"
        "${CMAKE_CURRENT_LIST_DIR}/examples/music/app")
    target_link_libraries(mr_music_host_tests PRIVATE mr_module_cloudmusic)
    set_target_properties(mr_music_host_tests PROPERTIES
        C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
    target_compile_options(mr_music_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -UNDEBUG)
    target_compile_options(mr_arm_module_host_tests PRIVATE -UNDEBUG)
    add_test(NAME mr_music_host_tests COMMAND mr_music_host_tests)

    add_executable(mr_component_c_host_tests
        tests/component/scalar_c_host_tests.c
        User/component/ahrs.c
        User/component/crc16.c
        User/component/filter/filter.c
        User/component/pid.c
    )
    target_link_libraries(mr_component_c_host_tests PRIVATE mr_component_math)
    set_target_properties(mr_component_c_host_tests PROPERTIES
        C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
    target_compile_options(mr_component_c_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors)
    if(UNIX)
        target_link_libraries(mr_component_c_host_tests PRIVATE m)
    endif()
    add_test(NAME mr_component_c_host_tests COMMAND mr_component_c_host_tests)

    add_executable(mr_storage_host_tests tests/component/storage_host_tests.cpp
        User/component/container/spsc_ringbuf.c User/component/container/latest_slot.c)
    target_link_libraries(mr_storage_host_tests PRIVATE mr_component_math)
    target_include_directories(mr_storage_host_tests BEFORE PRIVATE "${CMAKE_CURRENT_LIST_DIR}/tests/stubs/container")
    set_target_properties(mr_storage_host_tests PROPERTIES
        C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_storage_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -UNDEBUG
        "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti>")
    add_test(NAME mr_storage_host_tests COMMAND mr_storage_host_tests)

    add_executable(mr_robotics_core_host_tests tests/component/robotics_core_host_tests.cpp)
    target_link_libraries(mr_robotics_core_host_tests PRIVATE mr_robotics_core)
    set_target_properties(mr_robotics_core_host_tests PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_robotics_core_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti -UNDEBUG)
    add_test(NAME mr_robotics_core_host_tests COMMAND mr_robotics_core_host_tests)

    add_library(mr_numeric_compile_check OBJECT tests/component/embedded_numeric_compile_check.cpp)
    target_link_libraries(mr_numeric_compile_check PRIVATE mr_robotics_arm mr_component_controller)
    set_target_properties(mr_numeric_compile_check PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_numeric_compile_check PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti)

    add_executable(mr_numeric_host_tests tests/component/numeric_host_tests.cpp)
    target_link_libraries(mr_numeric_host_tests PRIVATE mr_component_math)
    set_target_properties(mr_numeric_host_tests PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_numeric_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti -UNDEBUG)
    add_test(NAME mr_numeric_host_tests COMMAND mr_numeric_host_tests)

    add_executable(mr_component_cpp_host_tests
        tests/component/component_cpp_host_tests.cpp)
    target_link_libraries(mr_component_cpp_host_tests PRIVATE mr_component_math)
    set_target_properties(mr_component_cpp_host_tests PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_component_cpp_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti)
    add_test(NAME mr_component_cpp_host_tests COMMAND mr_component_cpp_host_tests)

    add_executable(mr_rm_command_group_host_tests
        tests/device/rm_command_group_host_tests.cpp)
    target_link_libraries(mr_rm_command_group_host_tests PRIVATE
        mr_component_math)
    set_target_properties(mr_rm_command_group_host_tests PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_rm_command_group_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti)
    add_test(NAME mr_rm_command_group_host_tests
             COMMAND mr_rm_command_group_host_tests)

    add_executable(mr_rm_protocol_host_tests
        tests/device/rm_protocol_host_tests.cpp
        User/device/motor/protocol/rm_protocol.cpp
)
    target_include_directories(mr_rm_protocol_host_tests BEFORE PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/tests/stubs")
    target_include_directories(mr_rm_protocol_host_tests PRIVATE "${CMAKE_CURRENT_LIST_DIR}/tests")
    target_link_libraries(mr_rm_protocol_host_tests PRIVATE mr_component_math)
    set_target_properties(mr_rm_protocol_host_tests PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_rm_protocol_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti)
    add_test(NAME mr_rm_protocol_host_tests COMMAND mr_rm_protocol_host_tests)

    add_executable(mr_dm_lz_protocol_host_tests
        tests/device/dm_lz_protocol_host_tests.cpp
        User/device/motor/protocol/dm_protocol.cpp
        User/device/motor/protocol/lz_protocol.cpp
)
    target_include_directories(mr_dm_lz_protocol_host_tests BEFORE PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/tests/stubs")
    target_link_libraries(mr_dm_lz_protocol_host_tests PRIVATE mr_component_math)
    set_target_properties(mr_dm_lz_protocol_host_tests PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_dm_lz_protocol_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti)
    add_test(NAME mr_dm_lz_protocol_host_tests
             COMMAND mr_dm_lz_protocol_host_tests)

    add_executable(mr_motor_feedback_host_tests
        tests/device/motor_feedback_host_tests.cpp
        User/device/motor/protocol/dm_protocol.cpp
        User/device/motor/protocol/lz_protocol.cpp)
    target_include_directories(mr_motor_feedback_host_tests PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/tests" "${CMAKE_CURRENT_LIST_DIR}/User")
    set_target_properties(mr_motor_feedback_host_tests PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_compile_options(mr_motor_feedback_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti -UNDEBUG)
    target_compile_options(mr_rm_protocol_host_tests PRIVATE -UNDEBUG)
    add_test(NAME mr_motor_feedback_host_tests COMMAND mr_motor_feedback_host_tests)

    add_executable(mr_mrlink_host_tests tests/device/mrlink_host_tests.c)
    target_link_libraries(mr_mrlink_host_tests PRIVATE mr_component_mrlink)
    target_include_directories(mr_mrlink_host_tests PRIVATE "${CMAKE_CURRENT_LIST_DIR}/User/device")
    set_target_properties(mr_mrlink_host_tests PROPERTIES
        C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
    target_compile_options(mr_mrlink_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors)
    add_test(NAME mr_mrlink_host_tests COMMAND mr_mrlink_host_tests)

    add_executable(mr_mrlink_cpp_host_tests tests/device/mrlink_cpp_host_tests.cpp)
    target_link_libraries(mr_mrlink_cpp_host_tests PRIVATE mr_component_mrlink)
    target_compile_options(mr_mrlink_cpp_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -fno-exceptions -fno-rtti -UNDEBUG)
    add_test(NAME mr_mrlink_cpp_host_tests COMMAND mr_mrlink_cpp_host_tests)

    add_executable(mr_mrlink_channel_host_tests
        tests/device/mrlink_channel_host_tests.c User/bsp/uart.c User/bsp/spi.c User/bsp/dma.c
        User/device/bmi088.c User/device/dr16.c
        User/bsp/mrlink/channel.c
        User/bsp/mrlink/channel_uart.c
        User/bsp/mrlink/channel_usb.c
        User/bsp/mrlink/channel_fdcan.c)
    target_include_directories(mr_mrlink_channel_host_tests BEFORE PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/tests/stubs/io")
    target_include_directories(mr_mrlink_channel_host_tests PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/User" "${CMAKE_CURRENT_LIST_DIR}/User/device")
    set_target_properties(mr_mrlink_channel_host_tests PROPERTIES
        C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
    target_compile_options(mr_mrlink_channel_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -UNDEBUG)
    add_test(NAME mr_mrlink_channel_host_tests COMMAND mr_mrlink_channel_host_tests)

    add_executable(mr_filter_host_tests tests/component/filter_host_tests.cpp
        User/component/filter/filter.c User/component/pid.c)
    add_executable(mr_time_host_tests tests/component/time_host_tests.cpp
        User/component/timer.cpp)
    add_executable(mr_bsp_time_host_tests tests/component/bsp_time_host_tests.c User/bsp/time.c)
    target_include_directories(mr_bsp_time_host_tests BEFORE PRIVATE tests/stubs/time User)
    target_compile_features(mr_bsp_time_host_tests PRIVATE c_std_11)
    target_compile_options(mr_bsp_time_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -pedantic-errors -UNDEBUG)
    add_test(NAME mr_bsp_time_host_tests COMMAND mr_bsp_time_host_tests)
    add_executable(mr_task_services_host_tests tests/module/task_services_host_tests.cpp
        User/component/ahrs.c)
    add_executable(mr_pid_c_host_tests tests/component/pid_c_host_tests.c)
    add_executable(mr_mixer_host_tests tests/component/mixer_host_tests.c User/component/mixer.c)
    add_executable(mr_pid_cpp_host_tests tests/component/pid_cpp_host_tests.cpp
        User/component/pid.c)
    foreach(_mr_test IN ITEMS mr_filter_host_tests mr_time_host_tests mr_task_services_host_tests
            mr_pid_c_host_tests mr_pid_cpp_host_tests mr_mixer_host_tests)
        target_link_libraries(${_mr_test} PRIVATE mr_component_math)
        set_target_properties(${_mr_test} PROPERTIES
            C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
            CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
        target_compile_options(${_mr_test} PRIVATE -Wall -Wextra -Wpedantic -pedantic-errors
            "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti>")
        if(UNIX)
            target_link_libraries(${_mr_test} PRIVATE m)
        endif()
        add_test(NAME ${_mr_test} COMMAND ${_mr_test})
    endforeach()

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
