# tests/navigation/CMakeLists.txt

# nav_add_gtest(test_errorcalculator_e2e
#     SRCS
#         test_errorcalculator_e2e.cpp
#     LIBS
#         ErrorCalculator CorrectionApplier IO
# )

# nav_add_gtest(test_errorcalculator_basic
#     SRCS
#         test_errorcalculator_basic.cpp
#     LIBS
#         ErrorCalculator IO
# )

# nav_add_gtest(test_errorcalculator_realdata_smoke
#     SRCS
#         test_errorcalculator_realdata_smoke.cpp
#     LIBS
#         ErrorCalculator IO CorrectionApplier
# )

# nav_add_gtest(test_errorcalculator_realdata
#     SRCS
#         test_errorcalculator_realdata.cpp
#     LIBS
#         ErrorCalculator IO CorrectionApplier
# )

nav_add_gtest(test_navigationtasksolver_basic
    SRCS
        test_navigationtasksolver_basic.cpp
    LIBS
        ErrorCalculator IO NavigationTaskSolver
)

# Базовый путь к данным тестов (в исходниках)
set(NAV_TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/tests/data)

# Пробрасываем путь в код тестов
add_compile_definitions(TEST_DATA_DIR="${NAV_TEST_DATA_DIR}")
