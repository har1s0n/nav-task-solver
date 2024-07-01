cmake_minimum_required(VERSION 3.14)

project(nav-task-solver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Попытка найти Qt6
find_package(Qt6 COMPONENTS Core QUIET)

if (NOT Qt6_FOUND)
    find_package(Qt5 COMPONENTS Core REQUIRED)
    set(QT_VERSION 5)
else()
    set(QT_VERSION 6)
endif()

add_subdirectory(src)
add_subdirectory(tests)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/data)

add_executable(nav-task-solver src/main.cpp)

if (QT_VERSION EQUAL 5)
    target_link_libraries(nav-task-solver
        GridGenerator
        Qt5::Core
    )
else()
    target_link_libraries(nav-task-solver
        GridGenerator
        Qt6::Core
    )
endif()

include(GNUInstallDirs)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/nav-task-solver-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/nav-task-solver-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nav-task-solver
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/nav-task-solver-config.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nav-task-solver
)

# Экспорт целей для использования другими проектами
install(EXPORT nav-task-solver-targets
    FILE nav-task-solver-targets.cmake
    NAMESPACE nav-task-solver::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nav-task-solver
)

# Экспорт целей для использования в системе сборки
export(EXPORT nav-task-solver-targets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/nav-task-solver-targets.cmake"
    NAMESPACE nav-task-solver::
)
