include(${PROJECT_SOURCE_DIR}/inavFramework/import.cmake)

message(STATUS "inavFramework_INCLUDE_DIRS = ${inavFramework_INCLUDE_DIRS}")

add_library(IO
    datamanager.cpp
    sbascorrectionreader.cpp
)

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


install(TARGETS IO
    EXPORT nav-task-solver-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
