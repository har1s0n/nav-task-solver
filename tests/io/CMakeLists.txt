nav_add_gtest(test_sbascorrectionstore_basic
  SRCS
    test_sbascorrectionstore_basic.cpp
  LIBS
    IO
  FOLDER "tests/io"
)

nav_add_gtest(test_sbascorrectionstore_edge
  SRCS
    test_sbascorrectionstore_edge.cpp
  LIBS
    IO
  FOLDER "tests/io"
)

# Базовый путь к данным тестов (в исходниках)
set(NAV_TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/tests/data)

# Пробрасываем путь в код тестов
add_compile_definitions(TEST_DATA_DIR="${NAV_TEST_DATA_DIR}")
