# Static LZ4, built from the pristine submodule at external/lz4 so it is baked
# into libusgscsm. STARDS uses it to (de)compress state files.

cmake_minimum_required(VERSION 3.10)

set(LZ4_SRC_DIR "${CMAKE_SOURCE_DIR}/external/lz4/lib")

if(NOT EXISTS "${LZ4_SRC_DIR}/lz4.c")
  message(FATAL_ERROR
    "LZ4 submodule sources not found at ${LZ4_SRC_DIR}. "
    "Run: git submodule update --init --recursive")
endif()

add_library(usgscsm_lz4 STATIC
    "${LZ4_SRC_DIR}/lz4.c"
    "${LZ4_SRC_DIR}/lz4hc.c"
    "${LZ4_SRC_DIR}/lz4frame.c"
    "${LZ4_SRC_DIR}/xxhash.c"
)

# PIC so the static lib links into the usgscsm shared library.
set_target_properties(usgscsm_lz4 PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    C_VISIBILITY_PRESET hidden)

# Consumers include <lz4.h> etc. from the submodule's lib directory.
target_include_directories(usgscsm_lz4 PUBLIC
    $<BUILD_INTERFACE:${LZ4_SRC_DIR}>)
