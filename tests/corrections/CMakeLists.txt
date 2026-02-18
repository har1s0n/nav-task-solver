# tests/corrections/CMakeLists.txt
nav_add_gtest(test_correctionapplier_basic
    SRCS
        test_correctionapplier_basic.cpp
    LIBS
        CorrectionApplier IO
    FOLDER "tests/corrections"
)

# Базовый путь к данным тестов (в исходниках)
set(NAV_TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/tests/data)

# Пробрасываем путь в код тестов
add_compile_definitions(TEST_DATA_DIR="${NAV_TEST_DATA_DIR}")
