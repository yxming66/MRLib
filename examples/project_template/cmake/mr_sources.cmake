# 在本板 CMakeLists.txt 中创建 Cube 目标后 include 本文件。
# 路径以本板目录为基准：firmware/<board>/，库位于 libraries/MRLib/。
set(MR_LIBRARY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../libraries/MRLib"
    CACHE PATH "Path to the MRLib checkout")
set(MRLIB_BUILD_HOST_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${MR_LIBRARY_ROOT}" mr_library)

# 默认只验证映射/反馈；完成 UART、DMA 内存策略后用 -DMR_EXAMPLE_ENABLE_DR16=ON。
option(MR_EXAMPLE_ENABLE_DR16 "Start the example DR16 receiver" OFF)
option(MR_EXAMPLE_ENABLE_IMU "Start the example BMI088 reader" OFF)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    app/app_init.cpp
    app/cmd_center.cpp
    app/chassis_task.cpp
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    APP_EXAMPLE_ENABLE_DR16=$<BOOL:${MR_EXAMPLE_ENABLE_DR16}>
    APP_EXAMPLE_ENABLE_IMU=$<BOOL:${MR_EXAMPLE_ENABLE_IMU}>)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
add_library(board_adapter OBJECT
    "${MR_LIBRARY_ROOT}/User/bsp/fdcan.c"
    "${MR_LIBRARY_ROOT}/User/bsp/time.c"
    "${MR_LIBRARY_ROOT}/User/bsp/mm.c"
    board/fdcan_endpoints.c
    board/time_source.c
    # 需要哪个外设就同时取消对应两行的注释，并在 main.c 登记。
    # "${MR_LIBRARY_ROOT}/User/bsp/gpio.c"
    # board/gpio_endpoints.c
    # "${MR_LIBRARY_ROOT}/User/bsp/spi.c"
    # board/spi_endpoints.c
    # "${MR_LIBRARY_ROOT}/User/bsp/pwm.c"
    # board/pwm_endpoints.c
)

if(MR_EXAMPLE_ENABLE_DR16)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        app/remote_control_task.cpp
        "${MR_LIBRARY_ROOT}/User/device/dr16.c")
    target_sources(board_adapter PRIVATE
        "${MR_LIBRARY_ROOT}/User/bsp/uart.c"
        board/uart_endpoints.c)
endif()
if(MR_EXAMPLE_ENABLE_IMU)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        app/imu_task.cpp
        "${MR_LIBRARY_ROOT}/User/device/bmi088.c")
    target_sources(board_adapter PRIVATE
        "${MR_LIBRARY_ROOT}/User/bsp/gpio.c"
        "${MR_LIBRARY_ROOT}/User/bsp/spi.c"
        board/gpio_endpoints.c
        board/spi_endpoints.c)
endif()
if(MR_EXAMPLE_ENABLE_DR16 OR MR_EXAMPLE_ENABLE_IMU)
    target_sources(board_adapter PRIVATE "${MR_LIBRARY_ROOT}/User/bsp/dma.c")
endif()
# IMU 启用时已加入 GPIO/SPI，无需再取消这些源码的注释。
# 手动启用 UART/SPI 时，确保 bsp/dma.c 已加入且只编译一次。
# UART 还需 bsp/uart.c 和 board/uart_endpoints.c。
target_include_directories(board_adapter PUBLIC "${MR_LIBRARY_ROOT}/User" "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(board_adapter PUBLIC stm32cubemx)
# 与 Cube 的 plain 写法保持一致，避免混用 PRIVATE 签名。
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx board_adapter mr::module_chassis mr::device_motor mr::device_wheel)
