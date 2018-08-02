cmake_minimum_required(VERSION 3.10)

# Link runTests with what we want to test and the GTest and pthread library
add_executable(runTests TestyMcTestFace.cpp)
target_link_libraries(runTests usgscsm ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)

gtest_discover_tests(runTests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
