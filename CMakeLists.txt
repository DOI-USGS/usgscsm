cmake_minimum_required(VERSION 3.10)
project(usgscsm VERSION 1.6.0 DESCRIPTION "usgscsm library")

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 11)

# Set a default build type if none was specified
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
endif()

# Use external or embedded dependencies
option(USGSCSM_EXTERNAL_DEPS "If the library should be built with external or embedded dependencies" OFF)

# To find JSON's config file
set (CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/cmake;${CMAKE_PREFIX_PATH}")

if(USGSCSM_EXTERNAL_DEPS)
  message(STATUS "Using external dependencies")
  # CSM API library
  find_path(CSM_INCLUDE_DIR NAMES "csm.h"
                            PATH_SUFFIXES "csm"
                            PATHS $ENV{CONDA_PREFIX}/include/)
  find_library(CSM_LIBRARY csmapi PATHS $ENV{CONDA_PREFIX}/lib)
  message("--Found external CSM Library: ${CSM_LIBRARY}")
  message("--Found external CSM Include Directory: ${CSM_INCLUDE_DIR}")

  # Nlohmann JSON
  find_package(nlohmann_json REQUIRED)

  # Eigen
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)
   
  # ALE
  find_package(ale REQUIRED)
  set(ALE_TARGET ale::ale)
else()
  message(STATUS "Using embedded dependencies")
  # CSM API library
  add_subdirectory(csm)
  set(CSM_INCLUDE_DIR /csm)
  set(CSM_LIBRARY csmapi)

  # Nlohmann JSON
  set(JSON_BuildTests OFF CACHE INTERNAL "")
  add_subdirectory(json)

  # ALE
  set(ALE_USE_EXTERNAL_EIGEN OFF)
  set(ALE_USE_EXTERNAL_JSON ON)
  set(ALE_BUILD_LOAD OFF)
  set(ALE_BUILD_TESTS OFF)
  add_subdirectory(ale)
  set(ALE_TARGET ale)

  # Use Eigen included with ALE
  add_library (Eigen3::Eigen ALIAS eigen)
  set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ale/eigen)
endif(USGSCSM_EXTERNAL_DEPS)

add_library(usgscsm SHARED
            src/UsgsAstroPlugin.cpp
            src/UsgsAstroFrameSensorModel.cpp
            src/UsgsAstroPushFrameSensorModel.cpp
            src/UsgsAstroLsSensorModel.cpp
            src/UsgsAstroSarSensorModel.cpp
            src/Distortion.cpp
            src/Utilities.cpp
            src/EigenUtilities.cpp)

set_target_properties(usgscsm PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)

set(USGSCSM_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/usgscsm"
                         "${CMAKE_CURRENT_SOURCE_DIR}/include"
                         "${EIGEN3_INCLUDE_DIR}")

# These will be copied upon installation to ${CMAKE_INSTALL_INCLUDEDIR}/include
set(USGSCSM_INSTALL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/usgscsm"
                                 "${CMAKE_CURRENT_SOURCE_DIR}/include/spdlog")

target_include_directories(usgscsm
                           PUBLIC
                           ${USGSCSM_INCLUDE_DIRS}
                           ${CSM_INCLUDE_DIR}
)

target_link_libraries(usgscsm
                      ${CSM_LIBRARY}
                      ${ALE_TARGET}
                      nlohmann_json::nlohmann_json)

add_executable(usgscsm_cam_test bin/usgscsm_cam_test.cc)
target_link_libraries(usgscsm_cam_test
    usgscsm
    ${CSM_LIBRARY})

install(TARGETS usgscsm LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${USGSCSM_INSTALL_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS usgscsm_cam_test RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Optional build tests
option (USGSCSM_BUILD_TESTS "Build tests" ON)
if(USGSCSM_BUILD_TESTS)

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
