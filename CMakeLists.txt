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

set(_mr_target_catalog_file "${CMAKE_CURRENT_LIST_DIR}/cmake/targets.json")
file(READ "${_mr_target_catalog_file}" _mr_target_catalog)
string(JSON _mr_target_catalog_version GET "${_mr_target_catalog}" version)
if(NOT _mr_target_catalog_version EQUAL 1)
    message(FATAL_ERROR "Unsupported target catalog version: ${_mr_target_catalog_version}")
endif()
string(JSON _mr_target_count LENGTH "${_mr_target_catalog}" targets)

# The catalog is the single source for exported targets and their public build
# dependencies. Both CMake and mrctl.py consume it.
foreach(_mr_index RANGE 0 ${_mr_target_count})
    if(_mr_index EQUAL _mr_target_count)
        break()
    endif()
    string(JSON _mr_name GET "${_mr_target_catalog}" targets ${_mr_index} name)
    string(JSON _mr_alias GET "${_mr_target_catalog}" targets ${_mr_index} alias)
    string(JSON _mr_type GET "${_mr_target_catalog}" targets ${_mr_index} type)
    string(JSON _mr_header GET "${_mr_target_catalog}" targets ${_mr_index} header)
    if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/User/${_mr_header}")
        message(FATAL_ERROR "Public header for ${_mr_name} does not exist: User/${_mr_header}")
    endif()
    if(_mr_type STREQUAL "INTERFACE")
        add_library(${_mr_name} INTERFACE)
        set(_mr_usage_scope INTERFACE)
    elseif(_mr_type STREQUAL "STATIC")
        string(JSON _mr_source_count LENGTH "${_mr_target_catalog}" targets ${_mr_index} sources)
        set(_mr_sources)
        foreach(_mr_source_index RANGE 0 ${_mr_source_count})
            if(_mr_source_index EQUAL _mr_source_count)
                break()
            endif()
            string(JSON _mr_source GET "${_mr_target_catalog}" targets ${_mr_index} sources ${_mr_source_index})
            list(APPEND _mr_sources "${CMAKE_CURRENT_LIST_DIR}/${_mr_source}")
        endforeach()
        string(JSON _mr_build_by_default ERROR_VARIABLE _mr_build_error
            GET "${_mr_target_catalog}" targets ${_mr_index} build_by_default)
        if(NOT _mr_build_error AND _mr_build_by_default)
            add_library(${_mr_name} STATIC ${_mr_sources})
        else()
            add_library(${_mr_name} STATIC EXCLUDE_FROM_ALL ${_mr_sources})
        endif()
        set(_mr_usage_scope PUBLIC)
    else()
        message(FATAL_ERROR "Unsupported type '${_mr_type}' for target ${_mr_name}")
    endif()

    add_library(${_mr_alias} ALIAS ${_mr_name})
    string(JSON _mr_include_count LENGTH "${_mr_target_catalog}" targets ${_mr_index} include_dirs)
    foreach(_mr_include_index RANGE 0 ${_mr_include_count})
        if(_mr_include_index EQUAL _mr_include_count)
            break()
        endif()
        string(JSON _mr_include_dir GET "${_mr_target_catalog}" targets ${_mr_index} include_dirs ${_mr_include_index})
        target_include_directories(${_mr_name} ${_mr_usage_scope}
            "${CMAKE_CURRENT_LIST_DIR}/${_mr_include_dir}")
    endforeach()
    string(JSON _mr_feature_count LENGTH "${_mr_target_catalog}" targets ${_mr_index} compile_features)
    foreach(_mr_feature_index RANGE 0 ${_mr_feature_count})
        if(_mr_feature_index EQUAL _mr_feature_count)
            break()
        endif()
        string(JSON _mr_feature GET "${_mr_target_catalog}" targets ${_mr_index} compile_features ${_mr_feature_index})
        target_compile_features(${_mr_name} ${_mr_usage_scope} ${_mr_feature})
    endforeach()
endforeach()

foreach(_mr_index RANGE 0 ${_mr_target_count})
    if(_mr_index EQUAL _mr_target_count)
        break()
    endif()
    string(JSON _mr_name GET "${_mr_target_catalog}" targets ${_mr_index} name)
    string(JSON _mr_type GET "${_mr_target_catalog}" targets ${_mr_index} type)
    if(_mr_type STREQUAL "INTERFACE")
        set(_mr_dependency_scope INTERFACE)
    else()
        set(_mr_dependency_scope PUBLIC)
    endif()
    string(JSON _mr_dependency_count LENGTH "${_mr_target_catalog}" targets ${_mr_index} dependencies)
    set(_mr_dependencies)
    foreach(_mr_dependency_index RANGE 0 ${_mr_dependency_count})
        if(_mr_dependency_index EQUAL _mr_dependency_count)
            break()
        endif()
        string(JSON _mr_dependency GET "${_mr_target_catalog}" targets ${_mr_index} dependencies ${_mr_dependency_index})
        if(NOT TARGET ${_mr_dependency})
            message(FATAL_ERROR "Target ${_mr_name} depends on unknown target ${_mr_dependency}")
        endif()
        list(APPEND _mr_dependencies ${_mr_dependency})
    endforeach()
    if(_mr_dependencies)
        target_link_libraries(${_mr_name} ${_mr_dependency_scope} ${_mr_dependencies})
    endif()
endforeach()

if(MR_TESTLIB_PROJECT_TARGETS)
    foreach(_mr_selected_target IN LISTS MR_TESTLIB_PROJECT_TARGETS)
        if(NOT TARGET ${_mr_selected_target})
            message(FATAL_ERROR "Project selected unknown MR_TestLib target: ${_mr_selected_target}")
        endif()
    endforeach()
endif()

if(MR_TESTLIB_BUILD_HOST_TESTS)
    enable_testing()

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
    add_executable(mr_pid_cpp_host_tests tests/component/pid_cpp_host_tests.cpp
        User/component/pid.c)
    foreach(_mr_test IN ITEMS mr_filter_host_tests mr_time_host_tests mr_task_services_host_tests
            mr_pid_c_host_tests mr_pid_cpp_host_tests)
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
