cmake_minimum_required(VERSION 3.10)
project(usgscsm VERSION 1.4.0 DESCRIPTION "usgscsm library")

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 11)

# Set a default build type if none was specified
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
endif()

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

  message("--Found CSM Library: ${CSM_LIBRARY}")
  message("--Found CSM Include Directory: ${CSM_INCLUDE_DIR}")
endif(BUILD_CSM)

add_library(usgscsm SHARED
            src/UsgsAstroPlugin.cpp
            src/UsgsAstroFrameSensorModel.cpp
            src/UsgsAstroLsSensorModel.cpp
            src/UsgsAstroSarSensorModel.cpp
            src/Distortion.cpp
	    src/Utilities.cpp)

set_target_properties(usgscsm PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)

set(USGSCSM_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/usgscsm"
                         "${CMAKE_CURRENT_SOURCE_DIR}/include/")

target_include_directories(usgscsm
                           PUBLIC
                           ${USGSCSM_INCLUDE_DIRS}
                           ${CSM_INCLUDE_DIR}
)

target_link_libraries(usgscsm
                      ${CSM_LIBRARY})

if(WIN32)
  install(TARGETS usgscsm RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR})
else()
  install(TARGETS usgscsm LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
install(DIRECTORY ${USGSCSM_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


# Optional build or link against CSM
option (BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)

  include(GoogleTest)
  include(cmake/gtest.cmake)

  # Setup for GoogleTest
  find_package (Threads)

  target_link_libraries(usgscsm
                        gtest ${CMAKE_THREAD_LIBS_INIT})
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()
