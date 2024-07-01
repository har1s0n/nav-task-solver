# include(FetchContent)
# FetchContent_Declare(
#   googletest
#   URL https://github.com/google/googletest/archive/refs/tags/release-1.11.0.zip
# )
# # Для Windows: предотвратить установку тестов GoogleTest
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# FetchContent_MakeAvailable(googletest)

# enable_testing()

# add_executable(
#   nav-task-solver-tests
#   test_main.cpp
# )

# target_link_libraries(
#   nav-task-solver-tests
#   GTest::gtest_main
#   #GridGenerator
# )

# include(GoogleTest)
# gtest_discover_tests(nav-task-solver-tests)
