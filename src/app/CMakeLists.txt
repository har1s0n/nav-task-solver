include(${PROJECT_SOURCE_DIR}/inavFramework/import.cmake)

set(public_headers
    application.h)
set(sources
    ${public_headers}
    application.cpp
    main.cpp)

add_executable(nav-task-solver ${sources})

target_include_directories(nav-task-solver
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${inavFramework_INCLUDE_DIRS}
)

target_link_libraries(nav-task-solver
    PRIVATE
        IO
        CorrectionApplier
        SatelliteSelector
        GridGenerator
        ErrorCalculator
        ${inavFramework_LIBRARIES}
)
