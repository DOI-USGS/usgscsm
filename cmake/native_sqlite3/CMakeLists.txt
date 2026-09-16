# Host build of the sqlite3 shell, to generate proj.db during a cross-compiled
# build where usgscsm_sqlite3_shell cannot run. SQLITE3_SRC_DIR (the vendored
# amalgamation dir) is passed in by the caller.

cmake_minimum_required(VERSION 3.10)
project(native_sqlite3 C)

if(NOT DEFINED SQLITE3_SRC_DIR)
  message(FATAL_ERROR "SQLITE3_SRC_DIR must be set to the sqlite amalgamation dir")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../sqlite3_defs.cmake")

add_executable(sqlite3 "${SQLITE3_SRC_DIR}/shell.c" "${SQLITE3_SRC_DIR}/sqlite3.c")
target_include_directories(sqlite3 PRIVATE "${SQLITE3_SRC_DIR}")
target_compile_definitions(sqlite3 PRIVATE ${USGSCSM_SQLITE3_DEFS})

# dl provides sqlite's optional extension loading on Unix; harmless if absent.
find_library(DL_LIB dl)
if(DL_LIB)
  target_link_libraries(sqlite3 PRIVATE ${DL_LIB})
endif()

install(TARGETS sqlite3 RUNTIME DESTINATION bin)
