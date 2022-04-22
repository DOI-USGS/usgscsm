cmake_minimum_required(VERSION 3.10)

# Link runCSMCameraModelTests with what we want to test and the GTest and pthread library
add_executable(runCSMCameraModelTests
               PluginTests.cpp
               FrameCameraTests.cpp
               LineScanCameraTests.cpp
               DistortionTests.cpp
               SarTests.cpp
               ISDParsingTests.cpp
               UtilitiesTests.cpp)
if(WIN32)
  option(CMAKE_USE_WIN32_THREADS_INIT "using WIN32 threads" ON)
  option(gtest_disable_pthreads "Disable uses of pthreads in gtest." ON)
  target_link_libraries(runCSMCameraModelTests usgscsm ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
else()
    target_link_libraries(runCSMCameraModelTests usgscsm ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
endif()

# Test the test_usgscsm_cam_test program. These tests will be generated
# by cmake and then executed in the current order. The second test
# uses as input the output of the first test.
# 1. Save the model state for an ISD camera model.
add_test(NAME test_usgscsm_cam_test_save_state
    COMMAND usgscsm_cam_test --model data/orbitalLineScan.json --output-model-state model_state.json
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
# 2. Load back the state and save it again.
add_test(NAME test_usgscsm_cam_test_load_state
    COMMAND usgscsm_cam_test --model model_state.json --output-model-state model_state2.json
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)

gtest_discover_tests(runCSMCameraModelTests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
