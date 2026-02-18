add_library(test_common STATIC
  test_main.cpp        # если не нужен — уберите и используйте GTest::gtest_main
  test_utils.cpp
)

target_include_directories(test_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_common PUBLIC Qt6::Core)
