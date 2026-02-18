include(${PROJECT_SOURCE_DIR}/inavFramework/import.cmake)

message(STATUS "inavFramework_INCLUDE_DIRS = ${inavFramework_INCLUDE_DIRS}")

set(public_headers
    datamanager.h
    sbascorrectionstore.h
    dcbparser.h
)
set(sources
    ${public_headers}
    datamanager.cpp
    sbascorrectionstore.cpp
    dcbparser.cpp
)

add_library(IO ${sources})

target_include_directories(IO
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/modules/IO>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/inavFramework>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/inavFramework/include>
    $<INSTALL_INTERFACE:include>  # на будущее (при install)
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src     # если где-то внутри .cpp вы инклюдите через "modules/.."
)

if (QT_VERSION EQUAL 5)
    target_link_libraries(IO
        PRIVATE
            ${inavFramework_LIBRARIES}
        PUBLIC
            Qt5::Core
    )
else()
    target_link_libraries(IO
        PRIVATE
            ${inavFramework_LIBRARIES}
        PUBLIC
            Qt6::Core
    )
endif()

message(STATUS "inavFramework_LIBRARIES = ${inavFramework_LIBRARIES}")


install(TARGETS IO
    EXPORT nav-task-solver-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
