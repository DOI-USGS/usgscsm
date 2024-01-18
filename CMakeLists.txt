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

  # First configure proj library
  set(BUILD_SHARED_LIBS OFF)
  set(PROJ_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/PROJ")
  set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/PROJ/cmake ${CMAKE_MODULE_PATH})
  set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/PROJ/src ${CMAKE_MODULE_PATH})

  ################################################################################
  # Check for sqlite3
  ################################################################################

  find_program(EXE_SQLITE3 sqlite3)
  if(NOT EXE_SQLITE3)
    message(SEND_ERROR "sqlite3 binary not found!")
  endif()

  find_package(Sqlite3 REQUIRED)
  if(NOT SQLITE3_FOUND)
    message(SEND_ERROR "sqlite3 dependency not found!")
  endif()

  # Would build and run with older versions, but with horrible performance
  # See https://github.com/OSGeo/PROJ/issues/1718
  if("${SQLITE3_VERSION}" VERSION_LESS "3.11")
    message(SEND_ERROR "sqlite3 >= 3.11 required!")
  endif()

  include(ProjUtilities)
  include(Ccache)
  ################################################################################
  # ProjConfig.cmake - CMake build configuration of PROJ library
  ################################################################################
  # Copyright (C) 2010 Mateusz Loskot <mateusz@loskot.net>
  #
  # Distributed under the Boost Software License, Version 1.0.
  # (See accompanying file LICENSE_1_0.txt or copy at
  # https://www.boost.org/LICENSE_1_0.txt)
  ################################################################################
  include(CheckLibraryExists)
  include(CheckFunctionExists)

  # if C flags have -Werror, temporarily remove these while running some checks
  string(FIND "${CMAKE_C_FLAGS}" "-Werror" FIND_WERROR)
  if(FIND_WERROR GREATER_EQUAL 0)
    # we must be careful about not matching -Werror=something, so let's append
    # a space at the end of CMAKE_C_FLAGS and match -Werror with a trailing space
    string(REPLACE "-Werror " " " _tmp_CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ")
    set(_prev_CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    set(CMAKE_C_FLAGS "${_tmp_CMAKE_C_FLAGS}")
  endif()

  # check needed include file
  check_function_exists(localeconv HAVE_LOCALECONV)
  check_function_exists(strerror HAVE_STRERROR)
  if(NOT WIN32)
    check_library_exists(dl dladdr "" HAVE_LIBDL)
    check_library_exists(m exp "" HAVE_LIBM)
  endif()

  # restore CMAKE_C_FLAGS as before
  if(FIND_WERROR GREATER_EQUAL 0)
    set(CMAKE_C_FLAGS "${_prev_CMAKE_C_FLAGS}")
  endif()

  set(PACKAGE "proj")
  set(PACKAGE_BUGREPORT "https://github.com/OSGeo/PROJ/issues")
  set(PACKAGE_NAME "PROJ")
  set(PACKAGE_STRING "PROJ ${${PROJECT_NAME}_VERSION}")
  set(PACKAGE_TARNAME "proj")
  set(PACKAGE_URL "https://proj.org")
  set(PACKAGE_VERSION "${${PROJECT_NAME}_VERSION}")

  # check if a second proj_config.h exists (created by ./configure)
  # as this is within CMake's C_INCLUDES / CXX_INCLUDES
  set(AUTOCONF_PROJ_CONFIG_H "${PROJ_SOURCE_DIR}/src/proj_config.h")
  if(EXISTS ${AUTOCONF_PROJ_CONFIG_H})
    message(WARNING
      "Autoconf's ${AUTOCONF_PROJ_CONFIG_H} may interfere with this "
      "CMake build. Run 'make distclean' in the source directory "
      "before CMake's build.")
  endif()

  message("${CMAKE_CURRENT_SOURCE_DIR}")
  configure_file(${PROJ_SOURCE_DIR}/cmake/proj_config.cmake.in ${PROJ_SOURCE_DIR}/src/proj_config.h)
  include(ProjMac)
  include(policies)

  set(PROJ_DATA_PATH "${CMAKE_INSTALL_FULL_DATADIR}/usgscsm")

  include_directories(${PROJ_SOURCE_DIR}/src)
  # This line may need to be revisted once EISE work gets under way
  add_subdirectory(PROJ/data)

  ###################
  message(STATUS "Configuring proj library:")

  ##############################################
  ### SWITCH BETWEEN STATIC OR SHARED LIBRARY###
  ##############################################

  # default config is shared
  option(BUILD_SHARED_LIBS
    "Build PROJ library shared." ON)

  find_package(Threads QUIET)
  if(Threads_FOUND AND CMAKE_USE_PTHREADS_INIT)
    add_definitions(-DPROJ_HAS_PTHREADS)
  endif()

  option(ENABLE_IPO
    "Build library with interprocedural optimization (if available)." OFF)
  if(ENABLE_IPO)
    cmake_policy(SET CMP0069 NEW)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT ENABLE_IPO)
  endif()
  print_variable(ENABLE_IPO)


  ##############################################
  ###  library source list and include_list  ###
  ##############################################

  set(SRC_LIBPROJ_PROJECTIONS
    ${PROJ_SOURCE_DIR}/src/projections/aeqd.cpp
    ${PROJ_SOURCE_DIR}/src/projections/adams.cpp
    ${PROJ_SOURCE_DIR}/src/projections/gnom.cpp
    ${PROJ_SOURCE_DIR}/src/projections/laea.cpp
    ${PROJ_SOURCE_DIR}/src/projections/mod_ster.cpp
    ${PROJ_SOURCE_DIR}/src/projections/nsper.cpp
    ${PROJ_SOURCE_DIR}/src/projections/nzmg.cpp
    ${PROJ_SOURCE_DIR}/src/projections/ortho.cpp
    ${PROJ_SOURCE_DIR}/src/projections/stere.cpp
    ${PROJ_SOURCE_DIR}/src/projections/sterea.cpp
    ${PROJ_SOURCE_DIR}/src/projections/aea.cpp
    ${PROJ_SOURCE_DIR}/src/projections/bipc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/bonne.cpp
    ${PROJ_SOURCE_DIR}/src/projections/eqdc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/isea.cpp
    ${PROJ_SOURCE_DIR}/src/projections/ccon.cpp
    ${PROJ_SOURCE_DIR}/src/projections/imw_p.cpp
    ${PROJ_SOURCE_DIR}/src/projections/krovak.cpp
    ${PROJ_SOURCE_DIR}/src/projections/lcc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/poly.cpp
    ${PROJ_SOURCE_DIR}/src/projections/rpoly.cpp
    ${PROJ_SOURCE_DIR}/src/projections/sconics.cpp
    ${PROJ_SOURCE_DIR}/src/projections/rouss.cpp
    ${PROJ_SOURCE_DIR}/src/projections/cass.cpp
    ${PROJ_SOURCE_DIR}/src/projections/cc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/cea.cpp
    ${PROJ_SOURCE_DIR}/src/projections/eqc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/gall.cpp
    ${PROJ_SOURCE_DIR}/src/projections/labrd.cpp
    ${PROJ_SOURCE_DIR}/src/projections/som.cpp
    ${PROJ_SOURCE_DIR}/src/projections/merc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/mill.cpp
    ${PROJ_SOURCE_DIR}/src/projections/ocea.cpp
    ${PROJ_SOURCE_DIR}/src/projections/omerc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/somerc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/tcc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/tcea.cpp
    ${PROJ_SOURCE_DIR}/src/projections/times.cpp
    ${PROJ_SOURCE_DIR}/src/projections/tmerc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/tobmerc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/airy.cpp
    ${PROJ_SOURCE_DIR}/src/projections/aitoff.cpp
    ${PROJ_SOURCE_DIR}/src/projections/august.cpp
    ${PROJ_SOURCE_DIR}/src/projections/bacon.cpp
    ${PROJ_SOURCE_DIR}/src/projections/bertin1953.cpp
    ${PROJ_SOURCE_DIR}/src/projections/chamb.cpp
    ${PROJ_SOURCE_DIR}/src/projections/hammer.cpp
    ${PROJ_SOURCE_DIR}/src/projections/lagrng.cpp
    ${PROJ_SOURCE_DIR}/src/projections/larr.cpp
    ${PROJ_SOURCE_DIR}/src/projections/lask.cpp
    ${PROJ_SOURCE_DIR}/src/projections/latlong.cpp
    ${PROJ_SOURCE_DIR}/src/projections/nicol.cpp
    ${PROJ_SOURCE_DIR}/src/projections/ob_tran.cpp
    ${PROJ_SOURCE_DIR}/src/projections/oea.cpp
    ${PROJ_SOURCE_DIR}/src/projections/tpeqd.cpp
    ${PROJ_SOURCE_DIR}/src/projections/vandg.cpp
    ${PROJ_SOURCE_DIR}/src/projections/vandg2.cpp
    ${PROJ_SOURCE_DIR}/src/projections/vandg4.cpp
    ${PROJ_SOURCE_DIR}/src/projections/wag7.cpp
    ${PROJ_SOURCE_DIR}/src/projections/lcca.cpp
    ${PROJ_SOURCE_DIR}/src/projections/geos.cpp
    ${PROJ_SOURCE_DIR}/src/projections/boggs.cpp
    ${PROJ_SOURCE_DIR}/src/projections/collg.cpp
    ${PROJ_SOURCE_DIR}/src/projections/comill.cpp
    ${PROJ_SOURCE_DIR}/src/projections/crast.cpp
    ${PROJ_SOURCE_DIR}/src/projections/denoy.cpp
    ${PROJ_SOURCE_DIR}/src/projections/eck1.cpp
    ${PROJ_SOURCE_DIR}/src/projections/eck2.cpp
    ${PROJ_SOURCE_DIR}/src/projections/eck3.cpp
    ${PROJ_SOURCE_DIR}/src/projections/eck4.cpp
    ${PROJ_SOURCE_DIR}/src/projections/eck5.cpp
    ${PROJ_SOURCE_DIR}/src/projections/fahey.cpp
    ${PROJ_SOURCE_DIR}/src/projections/fouc_s.cpp
    ${PROJ_SOURCE_DIR}/src/projections/gins8.cpp
    ${PROJ_SOURCE_DIR}/src/projections/gstmerc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/gn_sinu.cpp
    ${PROJ_SOURCE_DIR}/src/projections/goode.cpp
    ${PROJ_SOURCE_DIR}/src/projections/igh.cpp
    ${PROJ_SOURCE_DIR}/src/projections/igh_o.cpp
    ${PROJ_SOURCE_DIR}/src/projections/imoll.cpp
    ${PROJ_SOURCE_DIR}/src/projections/imoll_o.cpp
    ${PROJ_SOURCE_DIR}/src/projections/hatano.cpp
    ${PROJ_SOURCE_DIR}/src/projections/loxim.cpp
    ${PROJ_SOURCE_DIR}/src/projections/mbt_fps.cpp
    ${PROJ_SOURCE_DIR}/src/projections/mbtfpp.cpp
    ${PROJ_SOURCE_DIR}/src/projections/mbtfpq.cpp
    ${PROJ_SOURCE_DIR}/src/projections/moll.cpp
    ${PROJ_SOURCE_DIR}/src/projections/nell.cpp
    ${PROJ_SOURCE_DIR}/src/projections/nell_h.cpp
    ${PROJ_SOURCE_DIR}/src/projections/patterson.cpp
    ${PROJ_SOURCE_DIR}/src/projections/putp2.cpp
    ${PROJ_SOURCE_DIR}/src/projections/putp3.cpp
    ${PROJ_SOURCE_DIR}/src/projections/putp4p.cpp
    ${PROJ_SOURCE_DIR}/src/projections/putp5.cpp
    ${PROJ_SOURCE_DIR}/src/projections/putp6.cpp
    ${PROJ_SOURCE_DIR}/src/projections/qsc.cpp
    ${PROJ_SOURCE_DIR}/src/projections/robin.cpp
    ${PROJ_SOURCE_DIR}/src/projections/s2.cpp
    ${PROJ_SOURCE_DIR}/src/projections/sch.cpp
    ${PROJ_SOURCE_DIR}/src/projections/sts.cpp
    ${PROJ_SOURCE_DIR}/src/projections/urm5.cpp
    ${PROJ_SOURCE_DIR}/src/projections/urmfps.cpp
    ${PROJ_SOURCE_DIR}/src/projections/wag2.cpp
    ${PROJ_SOURCE_DIR}/src/projections/wag3.cpp
    ${PROJ_SOURCE_DIR}/src/projections/wink1.cpp
    ${PROJ_SOURCE_DIR}/src/projections/wink2.cpp
    ${PROJ_SOURCE_DIR}/src/projections/healpix.cpp
    ${PROJ_SOURCE_DIR}/src/projections/natearth.cpp
    ${PROJ_SOURCE_DIR}/src/projections/natearth2.cpp
    ${PROJ_SOURCE_DIR}/src/projections/calcofi.cpp
    ${PROJ_SOURCE_DIR}/src/projections/eqearth.cpp
    ${PROJ_SOURCE_DIR}/src/projections/col_urban.cpp
  )

  set(SRC_LIBPROJ_CONVERSIONS
    ${PROJ_SOURCE_DIR}/src/conversions/axisswap.cpp
    ${PROJ_SOURCE_DIR}/src/conversions/cart.cpp
    ${PROJ_SOURCE_DIR}/src/conversions/geoc.cpp
    ${PROJ_SOURCE_DIR}/src/conversions/geocent.cpp
    ${PROJ_SOURCE_DIR}/src/conversions/noop.cpp
    ${PROJ_SOURCE_DIR}/src/conversions/topocentric.cpp
    ${PROJ_SOURCE_DIR}/src/conversions/set.cpp
    ${PROJ_SOURCE_DIR}/src/conversions/unitconvert.cpp
  )

  set(SRC_LIBPROJ_TRANSFORMATIONS
    ${PROJ_SOURCE_DIR}/src/transformations/affine.cpp
    ${PROJ_SOURCE_DIR}/src/transformations/deformation.cpp
    ${PROJ_SOURCE_DIR}/src/transformations/gridshift.cpp
    ${PROJ_SOURCE_DIR}/src/transformations/helmert.cpp
    ${PROJ_SOURCE_DIR}/src/transformations/hgridshift.cpp
    ${PROJ_SOURCE_DIR}/src/transformations/horner.cpp
    ${PROJ_SOURCE_DIR}/src/transformations/molodensky.cpp
    ${PROJ_SOURCE_DIR}/src/transformations/vgridshift.cpp
    ${PROJ_SOURCE_DIR}/src/transformations/xyzgridshift.cpp
    ${PROJ_SOURCE_DIR}/src/transformations/defmodel.cpp
    ${PROJ_SOURCE_DIR}/src/transformations/tinshift.cpp
    ${PROJ_SOURCE_DIR}/src/transformations/vertoffset.cpp
  )

  set(SRC_LIBPROJ_ISO19111
    ${PROJ_SOURCE_DIR}/src/iso19111/static.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/util.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/metadata.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/common.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/coordinates.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/crs.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/datum.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/coordinatesystem.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/io.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/internal.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/factory.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/c_api.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/operation/concatenatedoperation.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/operation/coordinateoperationfactory.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/operation/conversion.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/operation/esriparammappings.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/operation/oputils.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/operation/parammappings.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/operation/projbasedoperation.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/operation/singleoperation.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/operation/transformation.cpp
    ${PROJ_SOURCE_DIR}/src/iso19111/operation/vectorofvaluesparams.cpp
  )

  set(SRC_LIBPROJ_CORE
    ${PROJ_SOURCE_DIR}/src/4D_api.cpp
    ${PROJ_SOURCE_DIR}/src/aasincos.cpp
    ${PROJ_SOURCE_DIR}/src/adjlon.cpp
    ${PROJ_SOURCE_DIR}/src/auth.cpp
    ${PROJ_SOURCE_DIR}/src/ctx.cpp
    ${PROJ_SOURCE_DIR}/src/datum_set.cpp
    ${PROJ_SOURCE_DIR}/src/datums.cpp
    ${PROJ_SOURCE_DIR}/src/deriv.cpp
    ${PROJ_SOURCE_DIR}/src/dmstor.cpp
    ${PROJ_SOURCE_DIR}/src/ell_set.cpp
    ${PROJ_SOURCE_DIR}/src/ellps.cpp
    ${PROJ_SOURCE_DIR}/src/factors.cpp
    ${PROJ_SOURCE_DIR}/src/fwd.cpp
    ${PROJ_SOURCE_DIR}/src/gauss.cpp
    ${PROJ_SOURCE_DIR}/src/generic_inverse.cpp
    ${PROJ_SOURCE_DIR}/src/geodesic.c
    ${PROJ_SOURCE_DIR}/src/init.cpp
    ${PROJ_SOURCE_DIR}/src/initcache.cpp
    ${PROJ_SOURCE_DIR}/src/internal.cpp
    ${PROJ_SOURCE_DIR}/src/inv.cpp
    ${PROJ_SOURCE_DIR}/src/list.cpp
    ${PROJ_SOURCE_DIR}/src/log.cpp
    ${PROJ_SOURCE_DIR}/src/malloc.cpp
    ${PROJ_SOURCE_DIR}/src/mlfn.cpp
    ${PROJ_SOURCE_DIR}/src/msfn.cpp
    ${PROJ_SOURCE_DIR}/src/mutex.cpp
    ${PROJ_SOURCE_DIR}/src/param.cpp
    ${PROJ_SOURCE_DIR}/src/phi2.cpp
    ${PROJ_SOURCE_DIR}/src/pipeline.cpp
    ${PROJ_SOURCE_DIR}/src/pj_list.h
    ${PROJ_SOURCE_DIR}/src/pr_list.cpp
    ${PROJ_SOURCE_DIR}/src/proj_internal.h
    ${PROJ_SOURCE_DIR}/src/proj_mdist.cpp
    ${PROJ_SOURCE_DIR}/src/qsfn.cpp
    ${PROJ_SOURCE_DIR}/src/release.cpp
    ${PROJ_SOURCE_DIR}/src/rtodms.cpp
    ${PROJ_SOURCE_DIR}/src/strerrno.cpp
    ${PROJ_SOURCE_DIR}/src/strtod.cpp
    ${PROJ_SOURCE_DIR}/src/tsfn.cpp
    ${PROJ_SOURCE_DIR}/src/units.cpp
    ${PROJ_SOURCE_DIR}/src/wkt1_generated_parser.c
    ${PROJ_SOURCE_DIR}/src/wkt1_generated_parser.h
    ${PROJ_SOURCE_DIR}/src/wkt1_parser.cpp
    ${PROJ_SOURCE_DIR}/src/wkt1_parser.h
    ${PROJ_SOURCE_DIR}/src/wkt2_generated_parser.c
    ${PROJ_SOURCE_DIR}/src/wkt2_generated_parser.h
    ${PROJ_SOURCE_DIR}/src/wkt2_parser.cpp
    ${PROJ_SOURCE_DIR}/src/wkt2_parser.h
    ${PROJ_SOURCE_DIR}/src/wkt_parser.cpp
    ${PROJ_SOURCE_DIR}/src/wkt_parser.hpp
    ${PROJ_SOURCE_DIR}/src/zpoly1.cpp
    ${PROJ_SOURCE_DIR}/src/proj_json_streaming_writer.hpp
    ${PROJ_SOURCE_DIR}/src/proj_json_streaming_writer.cpp
    ${PROJ_SOURCE_DIR}/src/tracing.cpp
    ${PROJ_SOURCE_DIR}/src/grids.hpp
    ${PROJ_SOURCE_DIR}/src/grids.cpp
    ${PROJ_SOURCE_DIR}/src/filemanager.hpp
    ${PROJ_SOURCE_DIR}/src/filemanager.cpp
    ${PROJ_SOURCE_DIR}/src/networkfilemanager.cpp
    ${PROJ_SOURCE_DIR}/src/sqlite3_utils.hpp
    ${PROJ_SOURCE_DIR}/src/sqlite3_utils.cpp
    ${PROJ_SOURCE_DIR}/src/proj_config.h
  )

  set(HEADERS_LIBPROJ
    ${PROJ_SOURCE_DIR}/src/proj.h
    ${PROJ_SOURCE_DIR}/src/proj_experimental.h
    ${PROJ_SOURCE_DIR}/src/proj_constants.h
    ${PROJ_SOURCE_DIR}/src/proj_symbol_rename.h
    ${PROJ_SOURCE_DIR}/src/geodesic.h
  )

  # Group source files for IDE source explorers (e.g. Visual Studio)
  source_group("Header Files"
    FILES ${HEADERS_LIBPROJ})
  source_group("Source Files\\Core"
    FILES ${SRC_LIBPROJ_CORE})
  source_group("Source Files\\Conversions"
    FILES ${SRC_LIBPROJ_CONVERSIONS})
  source_group("Source Files\\Projections"
    FILES ${SRC_LIBPROJ_PROJECTIONS})
  source_group("Source Files\\Transformations"
    FILES ${SRC_LIBPROJ_TRANSFORMATIONS})
  source_group("Source Files\\ISO19111"
    FILES ${SRC_LIBPROJ_ISO19111})

  include_directories(${PROJ_SOURCE_DIR}/include)

  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  source_group("CMake Files" FILES CMakeLists.txt)

  # Embed PROJ_DATA data files location
  add_definitions(-DPROJ_DATA="${PROJ_DATA_PATH}")


  ###########################################################
  # targets to refresh wkt1_parser.cpp and wkt2_parser.cpp
  ###########################################################

  # Those targets need to be run manually each time wkt1_grammar.y / wkt2_grammar.y
  # is modified.
  # We could of course run them automatically, but that would make building
  # PROJ harder.

  # This target checks that wkt1_grammar.y md5sum has not changed
  # If it has, then it should be updated and the generate_wkt1_parser target
  # should be manually run
  add_custom_target(check_wkt1_grammar_md5 ALL
                    COMMAND ${CMAKE_COMMAND}
                        "-DIN_FILE=wkt1_grammar.y"
                        "-DTARGET=generate_wkt1_parser"
                        "-DEXPECTED_MD5SUM=5b4495c1ec6d2ae26b7028a9bb5d8819"
                        -P "${PROJ_SOURCE_DIR}/src/check_md5sum.cmake"
                    WORKING_DIRECTORY "${PROJ_SOURCE_DIR}/src"
                    DEPENDS "${PROJ_SOURCE_DIR}/src/wkt1_grammar.y"
                    VERBATIM)

  add_custom_target(generate_wkt1_parser
                    COMMAND ${CMAKE_COMMAND}
                        "-DPREFIX=pj_wkt1_"
                        "-DIN_FILE=wkt1_grammar.y"
                        "-DOUT_FILE=wkt1_generated_parser.c"
                        -P "${PROJ_SOURCE_DIR}/src/generate_wkt_parser.cmake"
                    WORKING_DIRECTORY "${PROJ_SOURCE_DIR}/src"
                    VERBATIM)

  # This target checks that wkt2_grammar.y md5sum has not changed
  # If it has, then it should be updated and the generate_wkt2_parser target
  # should be manually run
  add_custom_target(check_wkt2_grammar_md5 ALL
                    COMMAND ${CMAKE_COMMAND}
                        "-DIN_FILE=wkt2_grammar.y"
                        "-DTARGET=generate_wkt2_parser"
                        "-DEXPECTED_MD5SUM=289572eebe9dab3c7225bd48c445c287"
                        -P "${PROJ_SOURCE_DIR}/src/check_md5sum.cmake"
                    WORKING_DIRECTORY "${PROJ_SOURCE_DIR}/src"
                    DEPENDS "${PROJ_SOURCE_DIR}/src/wkt2_grammar.y"
                    VERBATIM)

  add_custom_target(generate_wkt2_parser
                    COMMAND ${CMAKE_COMMAND}
                        "-DPREFIX=pj_wkt2_"
                        "-DIN_FILE=wkt2_grammar.y"
                        "-DOUT_FILE=wkt2_generated_parser.c"
                        -P "${PROJ_SOURCE_DIR}/src/generate_wkt_parser.cmake"
                    WORKING_DIRECTORY "${PROJ_SOURCE_DIR}/src"
                    VERBATIM)

  #################################################
  ## targets: libproj and proj_config.h
  #################################################
  set(ALL_LIBPROJ_SOURCES
    ${SRC_LIBPROJ_CORE}
    ${SRC_LIBPROJ_CONVERSIONS}
    ${SRC_LIBPROJ_PROJECTIONS}
    ${SRC_LIBPROJ_TRANSFORMATIONS}
    ${SRC_LIBPROJ_ISO19111}
  )
  set(ALL_LIBPROJ_HEADERS ${HEADERS_LIBPROJ})

  # Configuration for the core target "proj"
  proj_target_output_name(proj PROJ_CORE_TARGET_OUTPUT_NAME)

  add_library(proj OBJECT
    ${ALL_LIBPROJ_SOURCES}
    ${ALL_LIBPROJ_HEADERS}
    ${PROJ_RESOURCES}
  )
  add_library(PROJ::proj ALIAS proj)

  if(MSVC OR MINGW)
      target_compile_definitions(proj PRIVATE -DNOMINMAX)
  endif()

  # Tell Intel compiler to do arithmetic accurately.  This is needed to stop the
  # compiler from ignoring parentheses in expressions like (a + b) + c and from
  # simplifying 0.0 + x to x (which is wrong if x = -0.0).
  if("${CMAKE_C_COMPILER_ID}" STREQUAL "Intel")
    if(MSVC)
      set(FP_PRECISE "/fp:precise")
    else()
      set(FP_PRECISE "-fp-model precise")
    endif()
    # Apply to source files that require this option
    set_source_files_properties(
      geodesic.c
      PROPERTIES COMPILE_FLAGS ${FP_PRECISE})
  endif()

  if(ENABLE_IPO)
    set_property(TARGET proj
      PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()

  target_include_directories(proj INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJ_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

  if(WIN32)
    set_target_properties(proj
      PROPERTIES
      VERSION "${PROJ_VERSION}"
      OUTPUT_NAME "${PROJ_CORE_TARGET_OUTPUT_NAME}"
      ARCHIVE_OUTPUT_NAME proj
      CLEAN_DIRECT_OUTPUT 1)
  elseif(BUILD_FRAMEWORKS_AND_BUNDLE)
    set_target_properties(proj
      PROPERTIES
      VERSION "${PROJ_VERSION}"
      INSTALL_NAME_DIR ${PROJ_INSTALL_NAME_DIR}
      CLEAN_DIRECT_OUTPUT 1)
  else()
    set_target_properties(proj
      PROPERTIES
      VERSION "${PROJ_BUILD_VERSION}"
      SOVERSION "${PROJ_SOVERSION}"
      CLEAN_DIRECT_OUTPUT 1)
  endif()

  set_target_properties(proj
    PROPERTIES
    LINKER_LANGUAGE CXX)

  ##############################################
  # Link properties
  ##############################################
  set(PROJ_LIBRARIES proj)
  if(UNIX)
    find_library(M_LIB m)
    if(M_LIB)
      target_link_libraries(proj PRIVATE -lm)
    endif()
    find_library(DL_LIB dl)
    if(DL_LIB)
      target_link_libraries(proj PRIVATE -ldl)
    endif()
  endif()
  if(Threads_FOUND AND CMAKE_USE_PTHREADS_INIT)
    target_link_libraries(proj PRIVATE ${CMAKE_THREAD_LIBS_INIT})
  endif()

  target_include_directories(proj PRIVATE ${SQLITE3_INCLUDE_DIR})
  target_link_libraries(proj PRIVATE ${SQLITE3_LIBRARY})

  if(NLOHMANN_JSON STREQUAL "external")
    target_compile_definitions(proj PRIVATE EXTERNAL_NLOHMANN_JSON)
    target_link_libraries(proj
      PRIVATE $<BUILD_INTERFACE:nlohmann_json::nlohmann_json>)
  endif()

  if(BUILD_SHARED_LIBS)
    if(MSVC)
      target_compile_definitions(proj PRIVATE PROJ_MSVC_DLL_EXPORT=1)
    endif()
  else()
    target_compile_definitions(proj PUBLIC PROJ_DLL=)
  endif()

  ##############################################
  # Core configuration summary
  ##############################################
  print_variable(PROJ_CORE_TARGET_OUTPUT_NAME)
  print_variable(BUILD_SHARED_LIBS)
  print_variable(PROJ_LIBRARIES)
  ###################

  set(PROJ_TARGET proj)
  set(PROJ_INCLUDE_DIR /PROJ)

  # Nlohmann JSON
  set(JSON_BuildTests OFF CACHE INTERNAL "")
  add_subdirectory(json)

  # ALE
  # Use Eigen included with ALE
  add_library (eigen INTERFACE)
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

  # set(ALE_HEADER_FILES ${ALE_BUILD_INCLUDE_DIR}/Rotation.h
  #                      ${ALE_BUILD_INCLUDE_DIR}/Util.h)
  
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
                           "${CMAKE_CURRENT_SOURCE_DIR}/PROJ/include"
  )
  
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fPIC" )
  set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -fPIC" )

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

