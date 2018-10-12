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
endif(BUILD_CSM)

add_library(usgscsm SHARED
            src/UsgsAstroPlugin.cpp
            src/UsgsAstroFrameSensorModel.cpp
            src/UsgsAstroLsSensorModel.cpp)

set_target_properties(usgscsm PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)

set(USGSCSM_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/usgscsm"
                         "${CMAKE_CURRENT_SOURCE_DIR}/include/json")

target_include_directories(usgscsm
                           PUBLIC
                           ${USGSCSM_INCLUDE_DIRS}
                           ${CSM_INCLUDE_DIR}
)

# Setup for GoogleTest

find_package (Threads)

target_link_libraries(usgscsm
                      ${CSM_LIBRARY}
                      gtest ${CMAKE_THREAD_LIBS_INIT})

if(WIN32)
  option(CMAKE_USE_WIN32_THREADS_INIT "using WIN32 threads" ON)
  option(gtest_disable_pthreads "Disable uses of pthreads in gtest." ON)
  install(TARGETS usgscsm
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(DIRECTORY ${USGSCSM_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
else()
  install(TARGETS usgscsm
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(DIRECTORY ${USGSCSM_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()


# Optional build or link against CSM
option (BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()
