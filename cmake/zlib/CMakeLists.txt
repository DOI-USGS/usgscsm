# Static zlib, built from the pristine submodule at external/zlib so it is baked
# into libusgscsm. STARDS uses it to (de)compress state files.

cmake_minimum_required(VERSION 3.10)

set(ZLIB_SRC_DIR "${CMAKE_SOURCE_DIR}/external/zlib")

if(NOT EXISTS "${ZLIB_SRC_DIR}/zlib.h")
  message(FATAL_ERROR
    "zlib submodule sources not found at ${ZLIB_SRC_DIR}. "
    "Run: git submodule update --init --recursive")
endif()

# The checked-in zconf.h leaves Z_HAVE_UNISTD_H undefined (zlib's own CMake build
# stamps it in), so gzlib.c's lseek is implicitly declared and clang 16+ errors.
# Probe and define it ourselves; PUBLIC because it also decides whether z_off_t is
# off_t, which is part of the gzseek/gztell signatures.
include(CheckIncludeFile)
check_include_file(unistd.h USGSCSM_ZLIB_HAVE_UNISTD_H)

# Core sources, plus the gz* helpers so STARDS can read gzip-wrapped streams.
add_library(usgscsm_zlib STATIC
    "${ZLIB_SRC_DIR}/adler32.c"
    "${ZLIB_SRC_DIR}/compress.c"
    "${ZLIB_SRC_DIR}/crc32.c"
    "${ZLIB_SRC_DIR}/deflate.c"
    "${ZLIB_SRC_DIR}/gzclose.c"
    "${ZLIB_SRC_DIR}/gzlib.c"
    "${ZLIB_SRC_DIR}/gzread.c"
    "${ZLIB_SRC_DIR}/gzwrite.c"
    "${ZLIB_SRC_DIR}/infback.c"
    "${ZLIB_SRC_DIR}/inffast.c"
    "${ZLIB_SRC_DIR}/inflate.c"
    "${ZLIB_SRC_DIR}/inftrees.c"
    "${ZLIB_SRC_DIR}/trees.c"
    "${ZLIB_SRC_DIR}/uncompr.c"
    "${ZLIB_SRC_DIR}/zutil.c"
)

# PIC so the static lib links into the usgscsm shared library (required on ELF).
set_target_properties(usgscsm_zlib PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    C_VISIBILITY_PRESET hidden)

if(USGSCSM_ZLIB_HAVE_UNISTD_H)
  target_compile_definitions(usgscsm_zlib PUBLIC Z_HAVE_UNISTD_H)
endif()

# Silence zlib's CRT deprecation warnings (fopen/sprintf) on vendored code.
if(MSVC)
  target_compile_definitions(usgscsm_zlib PRIVATE _CRT_SECURE_NO_WARNINGS _CRT_NONSTDC_NO_DEPRECATE)
endif()

# Consumers include <zlib.h>/<zconf.h> from the submodule root.
target_include_directories(usgscsm_zlib PUBLIC
    $<BUILD_INTERFACE:${ZLIB_SRC_DIR}>)
