cmake_minimum_required(VERSION 3.10)
project(usgscsm VERSION 0.0.1 DESCRIPTION "usgscsm library")

include(cmake/gtest.cmake)
include(GNUInstallDirs)
include(GoogleTest)

set(CMAKE_CXX_STANDARD 11)

# Optional build or link against CSM
option (BUILD_CSM "Build the CSM library" ON)
if(BUILD_CSM)
  add_subdirectory(csm)
  set(CSM_INCLUDE_DIR /csm)
  set(CSM_LIBRARY csmapi)
else()
  find_path(CSM_INCLUDE_DIR NAMES "csm.h"
			    PATH_SUFFIXES "csm" 
			    PATHS $ENV{CONDA_PREFIX}/include/)
  find_library(CSM_LIBRARY csmapi PATHS $ENV{CONDA_PREFIX}/lib)
  message(${CSM_INCLUDE_DIR})
  message(${CSM_LIBRARY})
  message($ENV{CONDA_PREFIX})
endif(BUILD_CSM)

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
