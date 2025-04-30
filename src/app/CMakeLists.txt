add_executable(nav-task-solver
    main.cpp
)

target_include_directories(nav-task-solver
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(nav-task-solver
    PRIVATE
        IO
)
