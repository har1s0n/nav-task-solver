include(${INAV_FRAMEWORK_PATH}/import.cmake)

set(lib_headers application.h)
set(lib_sources application.cpp ${lib_headers})

add_library(nav-task-solver-lib STATIC ${lib_sources})

target_include_directories(nav-task-solver-lib
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src
    PRIVATE
        ${inavFramework_INCLUDE_DIRS}
)

target_link_libraries(nav-task-solver-lib
    PUBLIC
        IO
        SatelliteSelector
        GridGenerator
        ErrorCalculator
        NavigationTaskSolver
    PRIVATE
        ${inavFramework_LIBRARIES}
)

# 3. Оставляем исполняемый файл для локальной (standalone) отладки
add_executable(nav-task-solver main.cpp)

# Линкуем локальный исполняемый файл с нашей новой библиотекой
target_link_libraries(nav-task-solver PRIVATE nav-task-solver-lib)
