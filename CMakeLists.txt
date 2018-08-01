cmake_minimum_required(VERSION 3.10)
project(usgscsm VERSION 0.0.1 DESCRIPTION "usgscsm library")

set(CMAKE_CXX_STANDARD 11)

include(GNUInstallDirs)
add_library(usgscsm SHARED 
            src/UsgsAstroFramePlugin.cpp
            src/UsgsAstroFrameSensorModel.cpp
            src/UsgsAstroLsPlugin.cpp
            src/UsgsAstroLsSensorModel.cpp
            src/UsgsAstroLsStateData.cpp)

set_target_properties(usgscsm PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)

find_library(CSM_LIBRARY csmapi REQUIRED HINTS $ENV{CSM_LIBRARY_PATH})
find_path(CSM_INCLUDE_DIR NAMES csm HINTS $ENV{CSM_INCLUDE_PATH})

target_include_directories(usgscsm PRIVATE
                           include/
                           include/usgscsm
                           ${CSM_INCLUDE_DIR}
)

target_link_libraries(usgscsm ${CSM_LIBRARY})

if(WIN32)
  install(TARGETS usgscsm
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
else()
  install(TARGETS usgscsm
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()