include(${PROJECT_SOURCE_DIR}/inavFramework/import.cmake)

set(public_headers
    gridgenerator.h
    )
set(sources
    ${public_headers}
    gridgenerator.cpp
    )

add_library(GridGenerator ${sources})

target_include_directories(GridGenerator
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${inavFramework_INCLUDE_DIRS}
)

if (QT_VERSION EQUAL 5)
    target_link_libraries(GridGenerator
        PRIVATE
            ${inavFramework_LIBRARIES}
        PUBLIC
            Qt5::Core
    )
else()
    target_link_libraries(GridGenerator
        PRIVATE
            ${inavFramework_LIBRARIES}
        PUBLIC
            Qt6::Core
    )
endif()

install(TARGETS GridGenerator
    EXPORT nav-task-solver-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
