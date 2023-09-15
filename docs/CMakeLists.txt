message(STATUS "Setting Up Docs")

find_package(Doxygen REQUIRED)
find_package(Sphinx REQUIRED)

# Find all the public headers
set(USGSCSM_BUILD_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/usgscsm")
file(GLOB_RECURSE USGSCSM_PUBLIC_HEADERS ${USGSCSM_BUILD_INCLUDE_DIR}/*.h)

set(DOXYGEN_INPUT_DIR ${PROJECT_SOURCE_DIR}/usgscsm/include/)
set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/xml/index.xml)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

message(STATUS "DOXYGEN_INPUT_DIR: " ${DOXYGEN_INPUT_DIR})
message(STATUS "DOXYGEN_OUTPUT_DIR: " ${DOXYGEN_OUTPUT_DIR})
message(STATUS "DOXYGEN_INDEX_FILE: " ${DOXYGEN_INDEX_FILE})
message(STATUS "DOXYFILE_IN: " ${DOXYFILE_IN})
message(STATUS "DOXYFILE_OUT: " ${DOXYFILE_OUT})

file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
                   DEPENDS ${USGSCSM_PUBLIC_HEADERS}
                   COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT} 
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   MAIN_DEPENDENCY Doxyfile
                   COMMENT "Generating docs"
                   VERBATIM)

add_custom_target(Doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE})

set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/source)
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/sphinx)
set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

set(SPHINX_RST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/source/index.rst)

add_custom_command(OUTPUT ${SPHINX_INDEX_FILE}
                   COMMAND
                     ${SPHINX_EXECUTABLE} -b html
                     # Tell Breathe where to find the Doxygen output
                     -Dbreathe_projects.SpiceQL=${DOXYGEN_OUTPUT_DIR}/xml
                   ${SPHINX_SOURCE} ${SPHINX_BUILD}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                   DEPENDS ${USGSCSM_PUBLIC_HEADERS}
                   # Other docs files you want to track should go here (or in some variable)
                   ${SPHINX_RST_FILES}
                   ${CMAKE_CURRENT_SOURCE_DIR}/../README.md # Docs insert the readme, so it's a dep
                   ${DOXYGEN_INDEX_FILE}
                   # MAIN_DEPENDENCY ${SPHINX_SOURCE}/conf.py
                   COMMENT "Generating documentation with Sphinx")


# Nice named target so we can run the job easily
add_custom_target(Sphinx ALL DEPENDS ${SPHINX_INDEX_FILE})

# Add an install target to install the docs
include(GNUInstallDirs)
install(DIRECTORY ${SPHINX_BUILD}
DESTINATION ${CMAKE_INSTALL_DOCDIR})

