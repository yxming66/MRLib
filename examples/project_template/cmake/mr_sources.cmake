# 在本板 CMakeLists.txt 中创建 Cube 目标后 include 本文件。
# 路径以本板目录为基准：firmware/<board>/，库位于 libraries/MR_TestLib/。
set(MR_LIBRARY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../libraries/MR_TestLib")
set(MR_TESTLIB_BUILD_HOST_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${MR_LIBRARY_ROOT}" mr_library)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    app/app_init.cpp
    app/app_channels.cpp
    app/chassis_task.cpp
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
add_library(board_adapter OBJECT
    "${MR_LIBRARY_ROOT}/User/bsp/fdcan.c"
    "${MR_LIBRARY_ROOT}/User/bsp/time.c"
    "${MR_LIBRARY_ROOT}/User/bsp/mm.c"
    board/fdcan_endpoints.c
    board/time_source.c
    # 需要哪个外设就同时取消对应两行的注释，并在 main.c 登记。
    # UART 或 SPI 任一启用时，还须取消下一行注释（只添加一次）。
    # "${MR_LIBRARY_ROOT}/User/bsp/dma.c"
    # "${MR_LIBRARY_ROOT}/User/bsp/uart.c"
    # board/uart_endpoints.c
    # "${MR_LIBRARY_ROOT}/User/bsp/gpio.c"
    # board/gpio_endpoints.c
    # "${MR_LIBRARY_ROOT}/User/bsp/spi.c"
    # board/spi_endpoints.c
    # "${MR_LIBRARY_ROOT}/User/bsp/pwm.c"
    # board/pwm_endpoints.c
)
target_include_directories(board_adapter PUBLIC "${MR_LIBRARY_ROOT}/User" "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(board_adapter PUBLIC stm32cubemx)
# 与 Cube 的 plain 写法保持一致，避免混用 PRIVATE 签名。
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx board_adapter mr::module_chassis mr::device_motor mr::device_wheel)
