add_library(GridGenerator GridGenerator.cpp)

target_include_directories(GridGenerator PRIVATE
    "${CMAKE_SOURCE_DIR}/include/GridGenerator"
    "${CMAKE_SOURCE_DIR}/data")

install(TARGETS GridGenerator
    EXPORT nav-task-solver-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
