include(${PROJECT_SOURCE_DIR}/inavFramework/import.cmake)

set(public_headers
    )
set(sources
    ${public_headers}
    )

add_library(ErrorCalculator ${sources}
    errorcalculator.h errorcalculator.cpp)

target_include_directories(ErrorCalculator
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/modules/NavigationSolver/ErrorCalculator>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/inavFramework>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/inavFramework/include>
        $<INSTALL_INTERFACE:include>
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
