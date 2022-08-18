cmake_minimum_required(VERSION 3.10)

set(CMAKE_CTEST_ARGUMENTS "--output-on-failure")

# Link runCSMCameraModelTests with what we want to test and the GTest and pthread library
add_executable(runCSMCameraModelTests
               PluginTests.cpp
               FrameCameraTests.cpp
               LineScanCameraTests.cpp
               PushFrameCameraTests.cpp
               DistortionTests.cpp
               SarTests.cpp
               ISDParsingTests.cpp
               UtilitiesTests.cpp
               EigenUtilitiesTests.cpp)
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
    COMMAND usgscsm_cam_test --model data/toughLroNacLineScan.json  --sample-rate 100 --subpixel-offset 0.3 --height-above-datum 2307.5 --desired-precision 1e-8 --output-model-state model_state.json
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
# 2. Load back the state and save it again.
add_test(NAME test_usgscsm_cam_test_load_state
    COMMAND usgscsm_cam_test --model model_state.json --output-model-state model_state2.json
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)


# 3. Save model state from .sup file
add_test(NAME test_usgscsm_cam_test_save_sup_state
    COMMAND usgscsm_cam_test --model data/gxp_model_file.sup --output-model-state gxp_model_state.json
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
# 4. Load model state
add_test(NAME test_usgscsm_cam_test_load_sup_state
    COMMAND usgscsm_cam_test --model gxp_model_state.json --output-model-state gxp_model_state2.json
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)

gtest_discover_tests(runCSMCameraModelTests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
