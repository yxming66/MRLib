# 普通 CMake 消费项目

不依赖 Python、YAML、HAL 或 RTOS。演示 add_subdirectory 引入库、链接公共 target、静态声明 Arm<3>/Chassis、执行底盘运动学与旋转算法，以及链接真实 C MRLink 到 C++ 应用。它验证接口、传递依赖和链接，不模拟机器人或初始化设备。

```sh
cmake -S examples/host_project -B build/manual-consumer -G Ninja
cmake --build build/manual-consumer
ctest --test-dir build/manual-consumer --output-on-failure
```

新建独立项目时复制 CMakeLists.txt 与 app，配置时传入 `-DMR_LIBRARY_ROOT=D:/path/to/MR_TestLib`；无需修改示例的 CMake 路径。路径含空格时对整个 `-D...` 参数加引号。也可将 CMake 中默认路径改为项目的 external/MR_TestLib，然后按需要替换 main 和 target_link_libraries。

CI 同时构建仓库内示例和复制到独立临时目录的示例。逐步接入见 [新项目接入](../../docs/NEW_PROJECT.md)，固件接入见 [板级集成](../../docs/BOARD_INTEGRATION.md)。
