include(${PROJECT_SOURCE_DIR}/inavFramework/import.cmake)

add_executable(nav-task-solver
    main.cpp
    application.cpp
)

target_include_directories(nav-task-solver
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${inavFramework_INCLUDE_DIRS}
)

target_link_libraries(nav-task-solver
    PRIVATE
        IO
        ${inavFramework_LIBRARIES}
)
