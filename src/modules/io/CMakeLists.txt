include(${PROJECT_SOURCE_DIR}/inavFramework/import.cmake)

message(STATUS "inavFramework_INCLUDE_DIRS = ${inavFramework_INCLUDE_DIRS}")

set(public_headers
    datamanager.h)
set(sources
    ${public_headers}
    datamanager.cpp)

add_library(IO ${sources}
    sbascorrectionstore.h sbascorrectionstore.cpp)

target_include_directories(IO
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${inavFramework_INCLUDE_DIRS}
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
