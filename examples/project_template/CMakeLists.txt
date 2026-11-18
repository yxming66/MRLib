# 使用方法：复制本文件为板级目录的 CMakeLists.txt。
# 目录约定：firmware/<board>/，公共库位于 libraries/MRLib/。
# 先用 CubeMX 生成本板的 Core/、Drivers/、Middlewares/、
# cmake/stm32cubemx/、toolchain、启动文件和链接脚本。
# toolchain 必须通过 CMakePresets.json 或 -DCMAKE_TOOLCHAIN_FILE 指定，
# 并与实际 MCU、浮点 ABI、链接脚本保持一致。

cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type")
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 新板修改这里；CubeMX 子目录使用 CMAKE_PROJECT_NAME 找到固件目标。
set(CMAKE_PROJECT_NAME xxxBoard)
project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)
add_executable(${CMAKE_PROJECT_NAME})

# CubeMX 管理的 HAL、RTOS、启动文件和 Core 源码，不在下方重复添加。
add_subdirectory(cmake/stm32cubemx)
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

set(MR_LIBRARY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../libraries/MRLib")
if(NOT EXISTS "${MR_LIBRARY_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "MRLib missing: run git submodule update --init --recursive")
endif()
set(MRLIB_BUILD_HOST_TESTS OFF CACHE BOOL "Build MRLib host tests" FORCE)
add_subdirectory("${MR_LIBRARY_ROOT}" mr_library)

# ===== 1. 应用源码：创建对应文件后，取消注释或继续添加 =====
set(APP_SOURCES
    # app/app_init.cpp
    # app/gimbal_task.cpp
    # app/gimbal_assembly.cpp
)

# ===== 2. 板级源码：BSP 实现 + 本板的硬件绑定，成套按需加入 =====
set(BOARD_SOURCES
    # CAN 电机需要的 BSP/端点，按实际依赖继续补充：
    # "${MR_LIBRARY_ROOT}/User/bsp/fdcan.c"
    # board/fdcan_endpoints.c

    # 库时基 + 本板时间源：
    # "${MR_LIBRARY_ROOT}/User/bsp/time.c"
    # board/time_source.c

    # UART：
    # "${MR_LIBRARY_ROOT}/User/bsp/uart.c"
    # "${MR_LIBRARY_ROOT}/User/bsp/dma.c"
    # board/uart_endpoints.c

    # GPIO：
    # "${MR_LIBRARY_ROOT}/User/bsp/gpio.c"
    # board/gpio_endpoints.c
)

# ===== 3. 直接编译的设备源码：仅添加未由所选库 target 管理的文件 =====
set(EXTRA_DEVICE_SOURCES
    # "${MR_LIBRARY_ROOT}/User/device/dr16.c"
)

# ===== 4. 选择 MRLib 功能：按应用实际使用情况取消注释 =====
set(MR_TARGETS
    mr::component_math
    # mr::module_gimbal
    # mr::device_joint
    # mr::device_motor
    # mr::device_motor_controller
    # mr::module_chassis
    # mr::device_wheel
    # mr::component_mrlink
)

# ===== 下面通常不需要修改 =====
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_SOURCES}
    ${EXTRA_DEVICE_SOURCES}
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

if(BOARD_SOURCES)
    add_library(board_adapter OBJECT ${BOARD_SOURCES})
    target_include_directories(board_adapter PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${MR_LIBRARY_ROOT}/User"
    )
    # BSP 自己编译时也需要 MCU 宏和 HAL/CMSIS/RTOS 头文件路径。
    target_link_libraries(board_adapter PUBLIC stm32cubemx)
    target_compile_options(board_adapter PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(${CMAKE_PROJECT_NAME} board_adapter)
endif()

# CubeMX 对固件目标使用 plain 签名，此处保持一致（不加 PRIVATE）。
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${MR_TARGETS}
)

# 仅对手写应用源码启用附加警告，不强制应用到第三方 HAL。
if(APP_SOURCES)
    set_source_files_properties(${APP_SOURCES} PROPERTIES
        COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic"
    )
endif()

# 不递归扫描整个 MRLib；不重复添加已由 Cube/MRLib target 编译的源文件。
# CMake 链接不会自动完成外设登记、设备初始化或 RTOS 任务创建。
