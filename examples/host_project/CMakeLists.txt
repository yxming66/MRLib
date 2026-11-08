cmake_minimum_required(VERSION 3.22)
project(mr_manual_consumer LANGUAGES C CXX)

# Override when this project is copied outside the library repository.
set(MR_LIBRARY_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." CACHE PATH "MR_TestLib checkout")
if(NOT EXISTS "${MR_LIBRARY_ROOT}/cmake/MrTargets.cmake")
    message(FATAL_ERROR "Set MR_LIBRARY_ROOT to the MR_TestLib checkout; see README.md")
endif()
set(MR_TESTLIB_BUILD_HOST_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${MR_LIBRARY_ROOT}" mr_library)
add_executable(robot_app app/main.cpp)
target_link_libraries(robot_app PRIVATE mr::module_arm mr::module_chassis mr::component_mrlink)

enable_testing()
add_test(NAME robot_app COMMAND robot_app)
