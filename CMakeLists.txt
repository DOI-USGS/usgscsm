cmake_minimum_required(VERSION 3.10)
project(usgscsm VERSION 2.0.0 DESCRIPTION "usgscsm library")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

message(STATUS "CMake Module Path: " ${CMAKE_MODULE_PATH})

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
option(ENABLE_CURL "Set to build the curl components of proj" OFF)
option(ENABLE_TIFF "Set to build the TIFF components of proj" OFF)
option(BUILD_TESTING "Set to build the proj tests" OFF)
option(BUILD_APPS "Set to build the proj apps" OFF)

# To find JSON's config file
set (CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/cmake;${CMAKE_PREFIX_PATH}")

set(USGSCSM_SRC_FILES src/UsgsAstroPlugin.cpp
                      src/UsgsAstroPluginSupport.cpp
                      src/UsgsAstroFrameSensorModel.cpp
                      src/UsgsAstroPushFrameSensorModel.cpp
                      src/UsgsAstroLsSensorModel.cpp
                      src/UsgsAstroProjectedSensorModel.cpp
                      src/UsgsAstroSarSensorModel.cpp
                      src/Distortion.cpp
                      src/Utilities.cpp
                      src/EigenUtilities.cpp
)

set(USGSCSM_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/usgscsm"
                         "${CMAKE_CURRENT_SOURCE_DIR}/include"
                         "${CMAKE_CURRENT_SOURCE_DIR}"
)

if(USGSCSM_EXTERNAL_DEPS)
  message(STATUS "Using external dependencies")
  # CSM API library
  find_path(CSM_INCLUDE_DIR NAMES "csm.h"
                            PATH_SUFFIXES "csm"
                            PATHS $ENV{CONDA_PREFIX}/include/)
  find_library(CSM_LIBRARY csmapi PATHS $ENV{CONDA_PREFIX}/lib)
  message("-- Found external CSM Library: ${CSM_LIBRARY}")
  message("-- Found external CSM Include Directory: ${CSM_INCLUDE_DIR}")

  # Nlohmann JSON
  find_package(nlohmann_json REQUIRED)

  # Eigen
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)

  # ALE
  find_package(ale REQUIRED)
  set(ALE_TARGET ale::ale)

  # Proj
  find_package(PROJ REQUIRED CONFIG)
  set(PROJ_TARGET PROJ::proj)
  add_library(usgscsm SHARED ${USGSCSM_SRC_FILES})
else()
  message(STATUS "Using embedded dependencies")
  # CSM API library
  add_subdirectory(csm)
  set(CSM_INCLUDE_DIR /csm)
  set(CSM_LIBRARY csmapi)

  add_subdirectory(PROJ)
  set(PROJ_INCLUDE_DIR /PROJ)
  set(PROJ_TARGET PROJ::proj)

  # Nlohmann JSON
  set(JSON_BuildTests OFF CACHE INTERNAL "")
  add_subdirectory(json)

  # ALE
  set(ALE_USE_EXTERNAL_EIGEN OFF)
  set(ALE_USE_EXTERNAL_JSON OFF)
  set(ALE_BUILD_LOAD OFF)
  set(ALE_BUILD_TESTS OFF)
  set(ALE_BUILD_DOCS OFF)

  # Use Eigen included with ALE
  add_library (Eigen3::Eigen ALIAS eigen)
  target_include_directories (eigen INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/ale/eigen)
  set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ale/eigen)

  set(ALE_BUILD_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ale/include/ale")

  set(ALE_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/ale/src/InterpUtils.cpp
                       ${CMAKE_CURRENT_SOURCE_DIR}/ale/src/Rotation.cpp
                       ${CMAKE_CURRENT_SOURCE_DIR}/ale/src/Orientations.cpp
                       ${CMAKE_CURRENT_SOURCE_DIR}/ale/src/States.cpp
                       ${CMAKE_CURRENT_SOURCE_DIR}/ale/src/Util.cpp
                       ${CMAKE_CURRENT_SOURCE_DIR}/ale/src/Vectors.cpp)

  set(ALE_HEADER_FILES ${ALE_BUILD_INCLUDE_DIR}/Rotation.h
                       ${ALE_BUILD_INCLUDE_DIR}/Util.h)
  
  add_library(ale OBJECT ${ALE_SOURCE_FILES})
  set(ALE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/ale/include/ale/")

  target_include_directories(ale
                            PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ale/include>
                            $<INSTALL_INTERFACE:include>
                            PRIVATE)

  target_include_directories(ale
                             PRIVATE
                             ${ALE_INCLUDE_DIRS}
  )
  set(ALE_LINKS nlohmann_json::nlohmann_json
                Eigen3::Eigen
  )
  target_link_libraries(ale PRIVATE ${ALE_LINKS})
  set(ALE_TARGET ale)
  set(USGSCSM_INCLUDE_DIRS "${USGSCSM_INCLUDE_DIRS}"
                           "${CMAKE_CURRENT_SOURCE_DIR}/ale/include"
  )
  add_library(usgscsm SHARED ${USGSCSM_SRC_FILES}
                             $<TARGET_OBJECTS:ale>
  )
endif(USGSCSM_EXTERNAL_DEPS)

set_target_properties(usgscsm PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)
set(USGSCSM_INCLUDE_DIRS "${USGSCSM_INCLUDE_DIRS}"
                         "${EIGEN3_INCLUDE_DIR}"
                         "${PROJ_INCLUDE_DIR}")

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
                      ${PROJ_TARGET}
                      nlohmann_json::nlohmann_json)

add_executable(usgscsm_cam_test bin/usgscsm_cam_test.cc)
target_link_libraries(usgscsm_cam_test
    usgscsm
    ${CSM_LIBRARY})

install(TARGETS usgscsm LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/csmplugins/)
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

option (USGSCSM_BUILD_DOCS "Build the USGSCSM Docs" ON)
if(USGSCSM_BUILD_DOCS)
  add_subdirectory ("docs")
else()
  message(STATUS "Skipping Docs")
endif()

