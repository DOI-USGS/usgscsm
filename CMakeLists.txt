cmake_minimum_required(VERSION 3.10)
project(usgscsm VERSION 0.0.1 DESCRIPTION "usgscsm library")

include(cmake/gtest.cmake)
include(GNUInstallDirs)
include(GoogleTest)

set(CMAKE_CXX_STANDARD 11)

add_library(usgscsm SHARED
            src/UsgsAstroFramePlugin.cpp
            src/UsgsAstroFrameSensorModel.cpp
            src/UsgsAstroLsPlugin.cpp
            src/UsgsAstroLsSensorModel.cpp
            src/UsgsAstroLsStateData.cpp)

set_target_properties(usgscsm PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    UsgsAstroFramePlugin.h
    UsgsAstroFrameSensorModel.h
    UsgsAstroLsISD.h
    UsgsAstroLsPlugin.h
    UsgsAstroLsSensorModel.h
    UsgsAstroLsStateData.h
)

find_library(CSM_LIBRARY csmapi REQUIRED HINTS $ENV{CSM_LIBRARY_PATH})
find_path(CSM_INCLUDE_DIR PATH_SUFFIXES csm NAMES csm.h HINTS $ENV{CSM_INCLUDE_PATH})

target_include_directories(usgscsm
                           PUBLIC
                           include/usgscsm
                           include/
                           ${CSM_INCLUDE_DIR}
)

# Setup for GoogleTest

find_package (Threads)

target_link_libraries(usgscsm
                      ${CSM_LIBRARY} 
                      gtest ${CMAKE_THREAD_LIBS_INIT})

if(WIN32)
  install(TARGETS usgscsm
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
else()
  install(TARGETS usgscsm
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()



include(CTest)

enable_testing()

add_subdirectory(tests)
