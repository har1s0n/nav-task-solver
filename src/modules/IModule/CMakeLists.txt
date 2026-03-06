#include(${PROJECT_SOURCE_DIR}/inavFramework/import.cmake)ƒ
include(${INAV_FRAMEWORK_PATH}/import.cmake)

set(public_headers
    imodule.h
    )
set(sources
    ${public_headers}
    )

add_library(IModule OBJECT ${sources})

target_include_directories(IModule
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/modules/IO>
    ${inavFramework_INCLUDE_DIRS}
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src
)

if (QT_VERSION EQUAL 5)
    target_link_libraries(IModule
        PRIVATE
            ${inavFramework_LIBRARIES}
        PUBLIC
            Qt5::Core
    )
else()
    target_link_libraries(IModule
        PRIVATE
            ${inavFramework_LIBRARIES}
        PUBLIC
            Qt6::Core
    )
endif()

install(TARGETS IModule
    EXPORT nav-task-solver-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
