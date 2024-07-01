cmake_minimum_required(VERSION 3.14)

project(nav-task-solver-tests LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/f8d7d77c06936315286eb55f8de22cd23c188571.zip
)

FetchContent_MakeAvailable(googletest)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(test_GridGenerator test_GridGenerator.cpp)
target_link_libraries(test_GridGenerator gtest_main GridGenerator)
add_test(NAME test_GridGenerator COMMAND test_GridGenerator)

#add_executable(test_class_b test_class_b.cpp)
#target_link_libraries(test_class_b gtest_main)
#add_test(NAME test_class_b COMMAND test_class_b)
