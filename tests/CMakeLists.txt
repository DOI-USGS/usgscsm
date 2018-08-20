cmake_minimum_required(VERSION 3.10)

# Link runCSMCameraModelTests with what we want to test and the GTest and pthread library
add_executable(runCSMCameraModelTests TestyMcTestFace.cpp FrameCameraTests.cpp)
target_link_libraries(runCSMCameraModelTests usgscsm ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)

gtest_discover_tests(runCSMCameraModelTests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
