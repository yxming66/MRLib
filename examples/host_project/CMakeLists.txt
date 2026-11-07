cmake_minimum_required(VERSION 3.22)
project(mr_manual_consumer LANGUAGES C CXX)

# In a separate repository replace ../.. with external/MR_TestLib.
set(MR_TESTLIB_BUILD_HOST_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../.." mr_library)
add_executable(robot_app app/main.cpp)
target_link_libraries(robot_app PRIVATE mr::module_arm mr::module_chassis mr::component_mrlink)

enable_testing()
add_test(NAME robot_app COMMAND robot_app)
