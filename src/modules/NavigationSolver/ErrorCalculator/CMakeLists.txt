include(${PROJECT_SOURCE_DIR}/inavFramework/import.cmake)

set(public_headers
    )
set(sources
    ${public_headers}
    )

add_library(ErrorCalculator ${sources})

target_include_directories(ErrorCalculator
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${inavFramework_INCLUDE_DIRS}
)

if (QT_VERSION EQUAL 5)
    target_link_libraries(ErrorCalculator
        PRIVATE
            ${inavFramework_LIBRARIES}
        PUBLIC
            Qt5::Core
    )
else()
    target_link_libraries(ErrorCalculator
        PRIVATE
            ${inavFramework_LIBRARIES}
        PUBLIC
            Qt6::Core
    )
endif()

install(TARGETS ErrorCalculator
    EXPORT nav-task-solver-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
