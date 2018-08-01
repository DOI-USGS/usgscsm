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
    PUBLIC_HEADER UsgsAstroFramePlugin.h
    PUBLIC_HEADER UsgsAstroFrameSensorModel.h
    PUBLIC_HEADER UsgsAstroLsISD.h
    PUBLIC_HEADER UsgsAstroLsPlugin.h
    PUBLIC_HEADER UsgsAstroLsSensorModel.h
    PUBLIC_HEADER UsgsAstroLsStateData.h
)

find_library(CSM_LIBRARY csmapi REQUIRED)
find_path(CSM_INCLUDE_DIR NAMES csm)

target_include_directories(usgscsm
                           PUBLIC
                           include/usgscsm
                           include/
                           ${CSM_INCLUDE_DIR}
)

target_link_libraries(usgscsm ${CSM_LIBRARY})

install(TARGETS usgscsm
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

include(CTest)

enable_testing()

add_subdirectory(tests)
