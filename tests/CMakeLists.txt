cmake_minimum_required(VERSION 3.10)

set(CMAKE_CTEST_ARGUMENTS "--output-on-failure")

# Link runCSMCameraModelTests with what we want to test and the GTest and pthread library
add_executable(runCSMCameraModelTests
               PluginTests.cpp
               FrameCameraTests.cpp
               LineScanCameraTests.cpp
               ProjectedCameraTests.cpp
               PushFrameCameraTests.cpp
               DistortionTests.cpp
               SarTests.cpp
               ISDParsingTests.cpp
               UtilitiesTests.cpp
               EigenUtilitiesTests.cpp
               ../ale/src/Rotation.cpp
               ../ale/src/InterpUtils.cpp)
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

# 5. Save binary (msgpack) model state from ISD.
add_test(NAME test_usgscsm_cam_test_save_binary_state
    COMMAND usgscsm_cam_test --model data/toughLroNacLineScan.json --sample-rate 100 --subpixel-offset 0.3 --height-above-datum 2307.5 --desired-precision 1e-8 --output-binary-model-state model_state_binary.isd
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
# 6. Load back the binary state.
add_test(NAME test_usgscsm_cam_test_load_binary_state
    COMMAND usgscsm_cam_test --model model_state_binary.isd
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)

# 7-8. Binary round-trip for frame model.
add_test(NAME test_usgscsm_cam_test_save_binary_frame
    COMMAND usgscsm_cam_test --model data/orbitalFramer.json --sample-rate 1 --output-binary-model-state frame_binary.isd
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
add_test(NAME test_usgscsm_cam_test_load_binary_frame
    COMMAND usgscsm_cam_test --model frame_binary.isd --sample-rate 1
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)

# 9-10. Binary round-trip for pushframe model.
add_test(NAME test_usgscsm_cam_test_save_binary_pushframe
    COMMAND usgscsm_cam_test --model data/orbitalPushFrame.json --sample-rate 100 --output-binary-model-state pushframe_binary.isd
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
add_test(NAME test_usgscsm_cam_test_load_binary_pushframe
    COMMAND usgscsm_cam_test --model pushframe_binary.isd --sample-rate 100
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)

# 11-12. Binary round-trip for SAR model.
add_test(NAME test_usgscsm_cam_test_save_binary_sar
    COMMAND usgscsm_cam_test --model data/orbitalSar.json --sample-rate 100000 --output-binary-model-state sar_binary.isd
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
add_test(NAME test_usgscsm_cam_test_load_binary_sar
    COMMAND usgscsm_cam_test --model sar_binary.isd --sample-rate 100000
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)

# 13-14. Binary round-trip for projected model (Mars linescan).
add_test(NAME test_usgscsm_cam_test_save_binary_projected
    COMMAND usgscsm_cam_test --model data/marsLineScanProj.json --sample-rate 1 --output-binary-model-state projected_binary.isd
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
add_test(NAME test_usgscsm_cam_test_load_binary_projected
    COMMAND usgscsm_cam_test --model projected_binary.isd --sample-rate 1
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)

# 15. Test the radtan model (this distortion model gets added to a DawnFC sensor)
add_test(NAME test_usgscsm_cam_test_radtan
    COMMAND usgscsm_cam_test --model data/dawnfc_radtan.json  --sample-rate 100 --output-model-state dawnfc_radtan_model_state.json
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
    
# The projected sensor test fixture (constVelocityProj.json) uses a sinusoidal
# projection with Mars radius (+R=3396190) but body radii of 10 m (synthetic
# test body). Newer PROJ versions reject this mismatch. The test geometry is
# intentionally synthetic, so suppress the check.
gtest_discover_tests(runCSMCameraModelTests
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests
  PROPERTIES ENVIRONMENT "PROJ_IGNORE_CELESTIAL_BODY=YES")
