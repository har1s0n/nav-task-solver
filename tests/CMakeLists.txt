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

# Если в тестах используешь Qt-типы (QString, QDateTime) — подтяни QtCore:
find_package(Qt6 REQUIRED COMPONENTS Core)

# Если нужны глобальные include тестов (обычно не нужно, если линкуем на цели проекта)
# include_directories(${CMAKE_SOURCE_DIR}/include)

# ---------- Универсальная функция для объявления gtest-таргета ----------
function(nav_add_gtest TARGET_NAME)
  set(options)
  set(oneValueArgs FOLDER)
  set(multiValueArgs SRCS LIBS DATA_DIRS)
  cmake_parse_arguments(NAVTEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  add_executable(${TARGET_NAME} ${NAVTEST_SRCS})

  target_link_libraries(${TARGET_NAME}
    PRIVATE
      GTest::gtest
      GTest::gtest_main
      Qt6::Core
      ${NAVTEST_LIBS}      # цели проекта: IO, Corrections, Navigation и т.п.
  )

  # (опционально) прокинуть пути к данным в define'ы
  foreach(dir IN LISTS NAVTEST_DATA_DIRS)
    target_compile_definitions(${TARGET_NAME}
      PRIVATE "TEST_DATA_DIR_${dir}=${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
  endforeach()

  add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})

  if(NAVTEST_FOLDER)
    set_target_properties(${TARGET_NAME} PROPERTIES FOLDER ${NAVTEST_FOLDER})
  endif()
endfunction()

# ---------- Подкаталоги тестов по модулям ----------
add_subdirectory(io)
add_subdirectory(corrections)
add_subdirectory(navigation)
