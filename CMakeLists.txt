cmake_minimum_required(VERSION 3.14)

project(nav-task-solver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Qt6 COMPONENTS Core Sql Network WebSockets QUIET)

if(NOT Qt6_FOUND)
    find_package(Qt5 COMPONENTS Core Sql Network WebSockets REQUIRED)
    set(QT_VERSION 5)
else()
    set(QT_VERSION 6)
endif()

# 1. Если путь не был передан явно через командную строку (-DINAV_FRAMEWORK_PATH=...)
if(NOT DEFINED INAV_FRAMEWORK_PATH)
    # Задаем список возможных расположений фреймворка (от наиболее вероятных к запасным)
    set(SEARCH_PATHS
        "${PROJECT_SOURCE_DIR}/../../../../../inavFramework" # 1. Интеграция внутри проекта ПОЭХ
        "${PROJECT_SOURCE_DIR}/inavFramework"                 # 2. Локальная папка (для standalone разработки)
        "${PROJECT_SOURCE_DIR}/../inavFramework"              # 3. Фреймворк лежит рядом с проектом
    )

    # Ищем первый существующий путь
    foreach(path IN LISTS SEARCH_PATHS)
        # Нормализуем путь (превращаем "../../.." в красивый абсолютный путь)
        get_filename_component(abs_path "${path}" ABSOLUTE)

        # Проверяем наличие целевого файла
        if(EXISTS "${abs_path}/import.cmake")
            set(INAV_FRAMEWORK_PATH "${abs_path}" CACHE PATH "Path to inavFramework" FORCE)
            message(STATUS "Auto-detected inavFramework at: ${INAV_FRAMEWORK_PATH}")
            break()
        endif()
    endforeach()
endif()

# 2. Строгая проверка перед подключением (защита от невнятных ошибок)
if(NOT EXISTS "${INAV_FRAMEWORK_PATH}/import.cmake")
    message(FATAL_ERROR "inavFramework/import.cmake not found! \
    Checked standard paths. Please set INAV_FRAMEWORK_PATH explicitly.")
endif()

# 3. Подключаем фреймворк
include("${INAV_FRAMEWORK_PATH}/import.cmake")

add_subdirectory(src)
add_subdirectory(tests)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/data)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/nav-task-solver-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/nav-task-solver-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nav-task-solver
)