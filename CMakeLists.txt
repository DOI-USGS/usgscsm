cmake_minimum_required(VERSION 3.10)
project(usgscsm VERSION 1.4.0 DESCRIPTION "usgscsm library")

include(GNUInstallDirs)

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

  message("--Found CSM Library: ${CSM_LIBRARY}")
  message("--Found CSM Include Directory: ${CSM_INCLUDE_DIR}")
endif(BUILD_CSM)

# Optional build or link against ALE
option (BUILD_ALE "Build the CSM library" ON)
if(BUILD_ALE)
  add_subdirectory(ale)
  set(ALE_INCLUDE_DIR /ale/include)
  set(ALE_LIBRARY ale)
else()
  find_path(ALE_INCLUDE_DIR NAMES "ale.h"
			    PATH_SUFFIXES "ale"
			    PATHS $ENV{CONDA_PREFIX}/include/)
  find_library(ALE_LIBRARY ale PATHS $ENV{CONDA_PREFIX}/lib)

  message("--Found CSM Library: ${ALE_LIBRARY}")
  message("--Found CSM Include Directory: ${ALE_INCLUDE_DIR}")
endif(BUILD_ALE)

add_library(usgscsm SHARED
            src/UsgsAstroPlugin.cpp
            src/UsgsAstroFrameSensorModel.cpp
            src/UsgsAstroLsSensorModel.cpp
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
                           ${ALE_INCLUDE_DIR}
)

target_link_libraries(usgscsm
                      ${CSM_LIBRARY}
                      ${ALE_LIBRARY})

if(WIN32)
  install(TARGETS usgscsm RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR})
else()
  install(TARGETS usgscsm LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
install(DIRECTORY ${USGSCSM_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


# Optional build or link against CSM
option (USGSCSM_BUILD_TESTS "Build tests" ON)
message("-- Build test var: ${USGSCSM_BUILD_TESTS}")
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
