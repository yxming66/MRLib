# Public library targets and direct dependencies. Maintain this file in CMake.
# Platform-specific BSP sources are supplied by the board project.

add_library(mr_component_mrlink STATIC EXCLUDE_FROM_ALL
    "${CMAKE_CURRENT_LIST_DIR}/../User/component/communication/mrlink/mrlink.c"
    "${CMAKE_CURRENT_LIST_DIR}/../User/component/crc16.c"
)
add_library(mr::component_mrlink ALIAS mr_component_mrlink)
target_include_directories(mr_component_mrlink PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_component_mrlink PUBLIC c_std_11 cxx_std_17)

add_library(mr_component_math INTERFACE)
add_library(mr::component_math ALIAS mr_component_math)
target_include_directories(mr_component_math INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_component_math INTERFACE c_std_11 cxx_std_17)

add_library(mr_component_filter INTERFACE)
add_library(mr::component_filter ALIAS mr_component_filter)
target_include_directories(mr_component_filter INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_component_filter INTERFACE cxx_std_17)

add_library(mr_component_controller INTERFACE)
add_library(mr::component_controller ALIAS mr_component_controller)
target_include_directories(mr_component_controller INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_component_controller INTERFACE cxx_std_17)

add_library(mr_component_trajectory INTERFACE)
add_library(mr::component_trajectory ALIAS mr_component_trajectory)
target_include_directories(mr_component_trajectory INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_component_trajectory INTERFACE cxx_std_17)

add_library(mr_robotics_core STATIC EXCLUDE_FROM_ALL
    "${CMAKE_CURRENT_LIST_DIR}/../User/robotics/core/rotation.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../User/robotics/core/transform.cpp"
)
add_library(mr::robotics_core ALIAS mr_robotics_core)
target_include_directories(mr_robotics_core PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_robotics_core PUBLIC cxx_std_17)

add_library(mr_robotics_chassis INTERFACE)
add_library(mr::robotics_chassis ALIAS mr_robotics_chassis)
target_include_directories(mr_robotics_chassis INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_robotics_chassis INTERFACE cxx_std_17)

add_library(mr_device_wheel_interface INTERFACE)
add_library(mr::device_wheel_interface ALIAS mr_device_wheel_interface)
target_include_directories(mr_device_wheel_interface INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_device_wheel_interface INTERFACE cxx_std_17)

add_library(mr_module_chassis INTERFACE)
add_library(mr::module_chassis ALIAS mr_module_chassis)
target_include_directories(mr_module_chassis INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_module_chassis INTERFACE cxx_std_17)

add_library(mr_device_joint_interface INTERFACE)
add_library(mr::device_joint_interface ALIAS mr_device_joint_interface)
target_include_directories(mr_device_joint_interface INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_device_joint_interface INTERFACE cxx_std_17)

add_library(mr_module_arm INTERFACE)
add_library(mr::module_arm ALIAS mr_module_arm)
target_include_directories(mr_module_arm INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_module_arm INTERFACE cxx_std_17)

add_library(mr_module_audio STATIC EXCLUDE_FROM_ALL
    "${CMAKE_CURRENT_LIST_DIR}/../User/module/audio/player.c"
)
add_library(mr::module_audio ALIAS mr_module_audio)
target_include_directories(mr_module_audio PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_module_audio PUBLIC c_std_11)

add_library(mr_module_cloudmusic STATIC EXCLUDE_FROM_ALL
    "${CMAKE_CURRENT_LIST_DIR}/../User/module/cloudmusic/cloudmusic.c"
)
add_library(mr::module_cloudmusic ALIAS mr_module_cloudmusic)
target_include_directories(mr_module_cloudmusic PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_module_cloudmusic PUBLIC c_std_11)

add_library(mr_robotics_arm INTERFACE)
add_library(mr::robotics_arm ALIAS mr_robotics_arm)
target_include_directories(mr_robotics_arm INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_robotics_arm INTERFACE cxx_std_17)

add_library(mr_component_timer STATIC EXCLUDE_FROM_ALL
    "${CMAKE_CURRENT_LIST_DIR}/../User/component/timer.cpp"
)
add_library(mr::component_timer ALIAS mr_component_timer)
target_include_directories(mr_component_timer PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_component_timer PUBLIC cxx_std_17)

add_library(mr_device_motor STATIC EXCLUDE_FROM_ALL
    "${CMAKE_CURRENT_LIST_DIR}/../User/device/motor/protocol/rm_protocol.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../User/device/motor/protocol/dm_protocol.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../User/device/motor/protocol/lz_protocol.cpp"
)
add_library(mr::device_motor ALIAS mr_device_motor)
target_include_directories(mr_device_motor PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_device_motor PUBLIC cxx_std_17)

add_library(mr_device_motor_controller STATIC EXCLUDE_FROM_ALL
    "${CMAKE_CURRENT_LIST_DIR}/../User/device/motor/packages/controller/motor_controller.cpp"
)
add_library(mr::device_motor_controller ALIAS mr_device_motor_controller)
target_include_directories(mr_device_motor_controller PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_device_motor_controller PUBLIC cxx_std_17)

add_library(mr_device_soft_limit_learning STATIC EXCLUDE_FROM_ALL
    "${CMAKE_CURRENT_LIST_DIR}/../User/device/motor/packages/soft_limit_learning/soft_limit_learning.cpp"
)
add_library(mr::device_soft_limit_learning ALIAS mr_device_soft_limit_learning)
target_include_directories(mr_device_soft_limit_learning PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_device_soft_limit_learning PUBLIC cxx_std_17)

add_library(mr_device_wheel INTERFACE)
add_library(mr::device_wheel ALIAS mr_device_wheel)
target_include_directories(mr_device_wheel INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_device_wheel INTERFACE cxx_std_17)

add_library(mr_device_joint INTERFACE)
add_library(mr::device_joint ALIAS mr_device_joint)
target_include_directories(mr_device_joint INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../User")
target_compile_features(mr_device_joint INTERFACE cxx_std_17)

# Link dependencies after all targets have been declared.

target_link_libraries(mr_component_filter INTERFACE
    mr_component_math
)

target_link_libraries(mr_component_controller INTERFACE
    mr_component_filter
    mr_component_math
)

target_link_libraries(mr_component_trajectory INTERFACE
    mr_component_math
)

target_link_libraries(mr_robotics_core PUBLIC
    mr_component_math
)

target_link_libraries(mr_robotics_chassis INTERFACE
    mr_component_math
)

target_link_libraries(mr_module_chassis INTERFACE
    mr_robotics_chassis
    mr_component_controller
    mr_device_wheel_interface
)

target_link_libraries(mr_module_arm INTERFACE
    mr_robotics_arm
    mr_device_joint_interface
)

target_link_libraries(mr_module_cloudmusic PUBLIC
    mr_module_audio
)

target_link_libraries(mr_robotics_arm INTERFACE
    mr_component_math
    mr_component_trajectory
    mr_robotics_core
)

target_link_libraries(mr_device_motor PUBLIC
    mr_component_math
)

target_link_libraries(mr_device_motor_controller PUBLIC
    mr_device_motor
    mr_component_controller
    mr_component_timer
)

target_link_libraries(mr_device_soft_limit_learning PUBLIC
    mr_component_math
    mr_component_timer
)

target_link_libraries(mr_device_wheel INTERFACE
    mr_device_wheel_interface
    mr_component_math
)

target_link_libraries(mr_device_joint INTERFACE
    mr_device_joint_interface
    mr_component_math
)
