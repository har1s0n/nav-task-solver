include(${INAV_FRAMEWORK_PATH}/import.cmake)

set(public_headers
    datamanager.h
    sbascorrectionstore.h
    dcbparser.h
    constellationstatusreader.h
)
set(sources
    ${public_headers}
    datamanager.cpp
    sbascorrectionstore.cpp
    dcbparser.cpp
    constellationstatusreader.cpp
)

add_library(IO OBJECT ${sources})

target_include_directories(IO
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/modules/IO>
    ${inavFramework_INCLUDE_DIRS}
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
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
