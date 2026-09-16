# Static SQLite3, vendored under upstream/ so it is baked into libusgscsm. Two
# targets: usgscsm_sqlite3 (linked into libusgscsm and PROJ) and
# usgscsm_sqlite3_shell (the CLI, used at build time as EXE_SQLITE3).

cmake_minimum_required(VERSION 3.10)

set(SQLITE3_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/upstream")

if(NOT EXISTS "${SQLITE3_SRC_DIR}/sqlite3.c")
  message(FATAL_ERROR
    "Vendored SQLite amalgamation not found at ${SQLITE3_SRC_DIR}. "
    "Run: git submodule update --init --recursive")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/sqlite3_defs.cmake")

# Static library compiled into libusgscsm and libproj.
add_library(usgscsm_sqlite3 STATIC "${SQLITE3_SRC_DIR}/sqlite3.c")
set_target_properties(usgscsm_sqlite3 PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    C_VISIBILITY_PRESET hidden)
target_compile_definitions(usgscsm_sqlite3 PRIVATE ${USGSCSM_SQLITE3_DEFS})
# Consumers (PROJ, the embedded VFS) include <sqlite3.h> from the upstream dir.
target_include_directories(usgscsm_sqlite3 PUBLIC
    $<BUILD_INTERFACE:${SQLITE3_SRC_DIR}>)

# Skipped under Emscripten: a cross-compiled shell cannot run on the host, so the
# WASM build supplies a native one separately.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
  add_executable(usgscsm_sqlite3_shell
      "${SQLITE3_SRC_DIR}/shell.c"
      "${SQLITE3_SRC_DIR}/sqlite3.c")
  target_compile_definitions(usgscsm_sqlite3_shell PRIVATE ${USGSCSM_SQLITE3_DEFS})
  target_include_directories(usgscsm_sqlite3_shell PRIVATE "${SQLITE3_SRC_DIR}")
  # dl is needed by the shell's default extension-loading support on Unix.
  if(UNIX)
    find_library(DL_LIB dl)
    if(DL_LIB)
      target_link_libraries(usgscsm_sqlite3_shell PRIVATE ${DL_LIB})
    endif()
  endif()
endif()
