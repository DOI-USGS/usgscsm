# Static CSM API from the pristine submodule at external/csm, whose own CMakeLists
# hardcodes SHARED. WASM only: a browser has no loader to share a csmapi through, so
# the module must carry its own. Native builds take the shared csmapi from conda
# instead -- see the csm::Plugin::theList note in the root CMakeLists.

cmake_minimum_required(VERSION 3.10)

set(CSM_SRC_DIR "${CMAKE_SOURCE_DIR}/external/csm")

if(NOT EXISTS "${CSM_SRC_DIR}/csm.h")
  message(FATAL_ERROR
    "CSM submodule sources not found at ${CSM_SRC_DIR}. "
    "Run: git submodule update --init --recursive")
endif()

add_library(csmapi STATIC
    "${CSM_SRC_DIR}/BundleGM.cpp"
    "${CSM_SRC_DIR}/CorrelationModel.cpp"
    "${CSM_SRC_DIR}/csmPointCloud.cpp"
    "${CSM_SRC_DIR}/Ellipsoid.cpp"
    "${CSM_SRC_DIR}/FourParameterCorrelationModel.cpp"
    "${CSM_SRC_DIR}/GeometricModel.cpp"
    "${CSM_SRC_DIR}/Isd.cpp"
    "${CSM_SRC_DIR}/LinearDecayCorrelationModel.cpp"
    "${CSM_SRC_DIR}/ModelIdentifier.cpp"
    "${CSM_SRC_DIR}/Plugin.cpp"
    "${CSM_SRC_DIR}/PointCloudGM.cpp"
    "${CSM_SRC_DIR}/PointCloudIsd.cpp"
    "${CSM_SRC_DIR}/RasterGM.cpp"
    "${CSM_SRC_DIR}/SettableEllipsoid.cpp"
    "${CSM_SRC_DIR}/Version.cpp"
)

# PIC so the static lib links into the usgscsm shared library.
set_target_properties(csmapi PROPERTIES POSITION_INDEPENDENT_CODE ON)

# PUBLIC: without CSM_LIBRARY, csm.h makes every symbol __declspec(dllimport) and
# MSVC looks for import stubs a static archive has none of.
target_compile_definitions(csmapi PUBLIC CSM_LIBRARY $<$<PLATFORM_ID:Linux>:LINUX_BUILD>)

target_include_directories(csmapi
    PRIVATE $<BUILD_INTERFACE:${CSM_SRC_DIR}>
    PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/external>)
