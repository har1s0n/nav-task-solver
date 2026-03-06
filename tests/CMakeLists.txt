cmake_minimum_required(VERSION 3.16)
project(nav-task-solver-tests LANGUAGES CXX)

include(CTest)
enable_testing()

include(FetchContent)

# --- CMP0135: включаем NEW поведение и/или задаём DOWNLOAD_EXTRACT_TIMESTAMP ---
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/f8d7d77c06936315286eb55f8de22cd23c188571.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(googletest)

find_package(Qt6 REQUIRED COMPONENTS Core)

set(NAV_ROOT_DIR "${nav-task-solver-tests_SOURCE_DIR}/..")

# ---------- Универсальная функция для объявления gtest-таргета ----------
function(nav_add_gtest TARGET_NAME)
  set(options)
  set(oneValueArgs FOLDER)
  set(multiValueArgs SRCS LIBS DATA_DIRS)
  cmake_parse_arguments(NAVTEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_executable(${TARGET_NAME} ${NAVTEST_SRCS})

  target_include_directories(${TARGET_NAME}
    PRIVATE
      ${NAV_ROOT_DIR}/src
  )

  target_link_libraries(${TARGET_NAME}
    PRIVATE
      GTest::gtest
      GTest::gtest_main
      Qt6::Core
      ${NAVTEST_LIBS}      # цели проекта: IO, Corrections, Navigation и т.п.
  )
endfunction()

# ---------- Подкаталоги тестов по модулям ----------
add_subdirectory(io)
add_subdirectory(navigation)
