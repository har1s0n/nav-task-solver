include(${INAV_FRAMEWORK_PATH}/import.cmake)

set(public_headers
    geometry.h
    weights.h
    solver.h
    navigationtasksolver.h
    )
set(sources
    ${public_headers}
    geometry.cpp
    weights.cpp
    solver.cpp
    navigationtasksolver.cpp
    )

add_library(NavigationTaskSolver OBJECT ${sources})

target_include_directories(NavigationTaskSolver
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/modules/IO>
    ${inavFramework_INCLUDE_DIRS}
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)

if (QT_VERSION EQUAL 5)
    target_link_libraries(NavigationTaskSolver
        PRIVATE
            ${inavFramework_LIBRARIES}
        PUBLIC
            Qt5::Core
    )
else()
    target_link_libraries(NavigationTaskSolver
        PRIVATE
            ${inavFramework_LIBRARIES}
        PUBLIC
            Qt6::Core
    )
endif()

install(TARGETS NavigationTaskSolver
    EXPORT nav-task-solver-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
