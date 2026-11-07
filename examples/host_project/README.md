# 普通 CMake 消费项目

不依赖 Python、YAML、HAL 或 RTOS。演示 add_subdirectory 引入库、链接公共 target、静态声明 Arm<3>/Chassis，以及链接真实 C MRLink 到 C++ 应用。它只验证接口和链接，不模拟机器人或初始化设备。

```sh
cmake -S examples/host_project -B build/manual-consumer -G Ninja
cmake --build build/manual-consumer
ctest --test-dir build/manual-consumer --output-on-failure
```

新建独立项目时复制 CMakeLists.txt 与 app，将库路径改为实际 external/MR_TestLib，再按需要替换 main 和 target_link_libraries。固件接入见 docs/BOARD_INTEGRATION.md。
