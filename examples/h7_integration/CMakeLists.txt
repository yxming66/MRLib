cmake_minimum_required(VERSION 3.22)
project(mr_h7_integration LANGUAGES C CXX ASM)
option(MR_CHASSIS_TASK_EXAMPLE "Build the standalone generated chassis task example" OFF)
if(MR_CHASSIS_TASK_EXAMPLE)
    set(_mr_default_config "${CMAKE_CURRENT_SOURCE_DIR}/../chassis_task/config.yaml")
else()
    set(_mr_default_config "${CMAKE_CURRENT_SOURCE_DIR}/config.yaml")
endif()
set(MR_ROBOT_CONFIG "${_mr_default_config}" CACHE FILEPATH "Robot YAML input")
set(MR_CONFIG_PYTHON "" CACHE FILEPATH "Python interpreter with config validation dependencies")
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(MR_CUBE_ROOT "" CACHE PATH "Read-only STM32H723 Cube project (Core, Drivers, Middlewares, startup)")
if(NOT EXISTS "${MR_CUBE_ROOT}/startup_stm32h723xx.s")
    message(FATAL_ERROR "Set MR_CUBE_ROOT to a complete STM32H723 Cube project; see README.md")
endif()
get_filename_component(MR_LIBRARY_ROOT "../.." ABSOLUTE)
set(_hal "${MR_CUBE_ROOT}/Drivers/STM32H7xx_HAL_Driver")
set(_rtos "${MR_CUBE_ROOT}/Middlewares/Third_Party/FreeRTOS/Source")

add_compile_options(-ffunction-sections -fdata-sections -Os -g3)
add_compile_definitions(STM32H723xx USE_HAL_DRIVER USE_PWR_LDO_SUPPLY)
include_directories(
    "${MR_CUBE_ROOT}/Core/Inc" "${_hal}/Inc"
    "${MR_CUBE_ROOT}/Drivers/CMSIS/Include"
    "${MR_CUBE_ROOT}/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
    "${_rtos}/include" "${_rtos}/CMSIS_RTOS_V2" "${_rtos}/portable/GCC/ARM_CM4F")

# Read the Cube files directly; never build its old main/freertos/User sources.
set(_cube_sources)
foreach(_name gpio dma fdcan spi tim usart stm32h7xx_it stm32h7xx_hal_msp
        stm32h7xx_hal_timebase_tim system_stm32h7xx syscalls sysmem)
    list(APPEND _cube_sources "${MR_CUBE_ROOT}/Core/Src/${_name}.c")
endforeach()
foreach(_name tim tim_ex cortex rcc rcc_ex flash flash_ex gpio hsem dma dma_ex mdma
        pwr pwr_ex i2c i2c_ex exti fdcan spi spi_ex uart uart_ex)
    list(APPEND _cube_sources "${_hal}/Src/stm32h7xx_hal_${_name}.c")
endforeach()
list(APPEND _cube_sources "${_hal}/Src/stm32h7xx_hal.c"
    "${MR_CUBE_ROOT}/startup_stm32h723xx.s")
foreach(_name croutine event_groups list queue stream_buffer tasks timers)
    list(APPEND _cube_sources "${_rtos}/${_name}.c")
endforeach()
list(APPEND _cube_sources "${_rtos}/CMSIS_RTOS_V2/cmsis_os2.c"
    "${_rtos}/portable/MemMang/heap_4.c" "${_rtos}/portable/GCC/ARM_CM4F/port.c")
add_library(cube_platform INTERFACE)
get_directory_property(_mr_cube_includes INCLUDE_DIRECTORIES)
target_include_directories(cube_platform INTERFACE ${_mr_cube_includes})
target_compile_definitions(cube_platform INTERFACE STM32H723xx USE_HAL_DRIVER USE_PWR_LDO_SUPPLY)
add_library(cube_sources OBJECT ${_cube_sources})
target_link_libraries(cube_sources PRIVATE cube_platform)

set(MR_TESTLIB_BUILD_HOST_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${MR_LIBRARY_ROOT}" mr_library)
include("${MR_LIBRARY_ROOT}/cmake/MrStm32H7.cmake")
mr_add_stm32h7_bsp(TARGET user_board PLATFORM cube_platform PROFILE h723_zzrobot
    COMPONENTS can mrlink)
target_compile_options(user_board PRIVATE -Wall -Wextra -Wpedantic -pedantic-errors -Werror)

# Compile every production User implementation, including optional peripherals.
# Separate objects: this proves syntax, not runtime coverage of unused features.
file(GLOB_RECURSE _user_sources CONFIGURE_DEPENDS
    "${MR_LIBRARY_ROOT}/User/*.c" "${MR_LIBRARY_ROOT}/User/*.cpp")
list(FILTER _user_sources EXCLUDE REGEX "/tests/")
add_library(user_all_compile_check OBJECT ${_user_sources})
target_include_directories(user_all_compile_check PRIVATE "${MR_LIBRARY_ROOT}/User")
target_compile_options(user_all_compile_check PRIVATE
    -Wall -Wextra -Wpedantic -pedantic-errors -Werror)

if(MR_CHASSIS_TASK_EXAMPLE)
    add_executable(mr_h7_integration board/main.c ../chassis_task/app/chassis_task.cpp)
    target_compile_definitions(mr_h7_integration PRIVATE MR_CHASSIS_TASK_EXAMPLE)
else()
    add_executable(mr_h7_integration board/main.c app/robot.cpp)
endif()
set_target_properties(mr_h7_integration PROPERTIES SUFFIX ".elf")
target_include_directories(mr_h7_integration PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(mr_h7_integration PRIVATE
    -Wall -Wextra -Wpedantic -pedantic-errors -Werror)
target_link_libraries(mr_h7_integration PRIVATE cube_platform cube_sources
    mr::component_mrlink mr::component_timer m)
if(MR_CHASSIS_TASK_EXAMPLE)
    mr_target_robot_config(TARGET mr_h7_integration CONFIG "${MR_ROBOT_CONFIG}"
        MODULE chassis TASK chassis BOARD_TARGET user_board PYTHON "${MR_CONFIG_PYTHON}")
else()
    mr_target_robot_config(TARGET mr_h7_integration CONFIG "${MR_ROBOT_CONFIG}"
        MODULE all BOARD_TARGET user_board PYTHON "${MR_CONFIG_PYTHON}")
endif()
target_link_options(mr_h7_integration PRIVATE
    "-T${CMAKE_CURRENT_SOURCE_DIR}/board/stm32h723.ld"
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections -Wl,--print-memory-usage
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/mr_h7_integration.map")
set_property(TARGET mr_h7_integration APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/board/stm32h723.ld")
find_program(MR_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(MR_SIZE arm-none-eabi-size REQUIRED)
add_custom_command(TARGET mr_h7_integration POST_BUILD
    COMMAND "${MR_OBJCOPY}" -O binary "$<TARGET_FILE:mr_h7_integration>"
        "${CMAKE_CURRENT_BINARY_DIR}/mr_h7_integration.bin"
    COMMAND "${MR_SIZE}" "$<TARGET_FILE:mr_h7_integration>" VERBATIM)
