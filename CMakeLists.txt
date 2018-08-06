cmake_minimum_required(VERSION 3.10)
project(usgscsm VERSION 0.0.1 DESCRIPTION "usgscsm library")

include(cmake/gtest.cmake)
include(GNUInstallDirs)
include(GoogleTest)

add_subdirectory(csm)

set(CMAKE_CXX_STANDARD 11)

add_library(usgscsm SHARED
            src/UsgsAstroFramePlugin.cpp
            src/UsgsAstroFrameSensorModel.cpp
            src/UsgsAstroLsPlugin.cpp
            src/UsgsAstroLsSensorModel.cpp
            src/UsgsAstroLsStateData.cpp)

file(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/include/usgscsm/*.h)
set_target_properties(usgscsm PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER ${HEADER_FILES}
)

target_include_directories(usgscsm
                           PUBLIC
                           include/usgscsm
                           include/
                           csm/}
)

# Setup for GoogleTest

find_package (Threads)

target_link_libraries(usgscsm
                      csmapi
                      gtest ${CMAKE_THREAD_LIBS_INIT})

if(WIN32)
  install(TARGETS usgscsm
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/usgscsm)
else()
  install(TARGETS usgscsm
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/usgscsm)
endif()



include(CTest)

enable_testing()

add_subdirectory(tests)
