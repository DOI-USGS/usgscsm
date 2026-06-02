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
               LoggingTests.cpp
               ../ale/src/Rotation.cpp
               ../ale/src/InterpUtils.cpp)
if(WIN32)
  option(CMAKE_USE_WIN32_THREADS_INIT "using WIN32 threads" ON)
  option(gtest_disable_pthreads "Disable uses of pthreads in gtest." ON)
  target_link_libraries(runCSMCameraModelTests usgscsm ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
else()
  target_link_libraries(runCSMCameraModelTests usgscsm ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
endif()

# Test the test_usgscsm_cam_test program. Output files are written to build directory
# with unique names to avoid race conditions during parallel test execution.

# 1-2. JSON model state round-trip
add_test(NAME test_usgscsm_cam_test_save_state
    COMMAND usgscsm_cam_test --model ${PROJECT_SOURCE_DIR}/tests/data/toughLroNacLineScan.json --sample-rate 100 --subpixel-offset 0.3 --height-above-datum 2307.5 --desired-precision 1e-8 --output-model-state ${CMAKE_CURRENT_BINARY_DIR}/test_save_state_output.json
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_test(NAME test_usgscsm_cam_test_load_state
    COMMAND usgscsm_cam_test --model ${CMAKE_CURRENT_BINARY_DIR}/test_save_state_output.json --output-model-state ${CMAKE_CURRENT_BINARY_DIR}/test_load_state_output.json
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 3-4. SUP file model state round-trip
add_test(NAME test_usgscsm_cam_test_save_sup_state
    COMMAND usgscsm_cam_test --model ${PROJECT_SOURCE_DIR}/tests/data/gxp_model_file.sup --output-model-state ${CMAKE_CURRENT_BINARY_DIR}/test_save_sup_output.json
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_test(NAME test_usgscsm_cam_test_load_sup_state
    COMMAND usgscsm_cam_test --model ${CMAKE_CURRENT_BINARY_DIR}/test_save_sup_output.json --output-model-state ${CMAKE_CURRENT_BINARY_DIR}/test_load_sup_output.json
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 5-6. Binary (msgpack) model state round-trip
add_test(NAME test_usgscsm_cam_test_save_binary_state
    COMMAND usgscsm_cam_test --model ${PROJECT_SOURCE_DIR}/tests/data/toughLroNacLineScan.json --sample-rate 100 --subpixel-offset 0.3 --height-above-datum 2307.5 --desired-precision 1e-8 --output-binary-model-state ${CMAKE_CURRENT_BINARY_DIR}/test_save_binary_state.isd
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_test(NAME test_usgscsm_cam_test_load_binary_state
    COMMAND usgscsm_cam_test --model ${CMAKE_CURRENT_BINARY_DIR}/test_save_binary_state.isd
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 7-8. Binary round-trip for frame model
add_test(NAME test_usgscsm_cam_test_save_binary_frame
    COMMAND usgscsm_cam_test --model ${PROJECT_SOURCE_DIR}/tests/data/orbitalFramer.json --sample-rate 1 --output-binary-model-state ${CMAKE_CURRENT_BINARY_DIR}/test_save_binary_frame.isd
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_test(NAME test_usgscsm_cam_test_load_binary_frame
    COMMAND usgscsm_cam_test --model ${CMAKE_CURRENT_BINARY_DIR}/test_save_binary_frame.isd --sample-rate 1
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 9-10. Binary round-trip for pushframe model
add_test(NAME test_usgscsm_cam_test_save_binary_pushframe
    COMMAND usgscsm_cam_test --model ${PROJECT_SOURCE_DIR}/tests/data/orbitalPushFrame.json --sample-rate 100 --output-binary-model-state ${CMAKE_CURRENT_BINARY_DIR}/test_save_binary_pushframe.isd
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_test(NAME test_usgscsm_cam_test_load_binary_pushframe
    COMMAND usgscsm_cam_test --model ${CMAKE_CURRENT_BINARY_DIR}/test_save_binary_pushframe.isd --sample-rate 100
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 11-12. Binary round-trip for SAR model
add_test(NAME test_usgscsm_cam_test_save_binary_sar
    COMMAND usgscsm_cam_test --model ${PROJECT_SOURCE_DIR}/tests/data/orbitalSar.json --sample-rate 100000 --output-binary-model-state ${CMAKE_CURRENT_BINARY_DIR}/test_save_binary_sar.isd
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_test(NAME test_usgscsm_cam_test_load_binary_sar
    COMMAND usgscsm_cam_test --model ${CMAKE_CURRENT_BINARY_DIR}/test_save_binary_sar.isd --sample-rate 100000
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 13-14. Binary round-trip for projected model (Mars linescan)
add_test(NAME test_usgscsm_cam_test_save_binary_projected
    COMMAND usgscsm_cam_test --model ${PROJECT_SOURCE_DIR}/tests/data/marsLineScanProj.json --sample-rate 1 --output-binary-model-state ${CMAKE_CURRENT_BINARY_DIR}/test_save_binary_projected.isd
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_test(NAME test_usgscsm_cam_test_load_binary_projected
    COMMAND usgscsm_cam_test --model ${CMAKE_CURRENT_BINARY_DIR}/test_save_binary_projected.isd --sample-rate 1
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 15. Test the radtan model (this distortion model gets added to a DawnFC sensor)
add_test(NAME test_usgscsm_cam_test_radtan
    COMMAND usgscsm_cam_test --model ${PROJECT_SOURCE_DIR}/tests/data/dawnfc_radtan.json --sample-rate 100 --output-model-state ${CMAKE_CURRENT_BINARY_DIR}/test_radtan_output.json
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# Set test dependencies to ensure load tests run after save tests complete
set_tests_properties(test_usgscsm_cam_test_load_state PROPERTIES DEPENDS test_usgscsm_cam_test_save_state)
set_tests_properties(test_usgscsm_cam_test_load_sup_state PROPERTIES DEPENDS test_usgscsm_cam_test_save_sup_state)
set_tests_properties(test_usgscsm_cam_test_load_binary_state PROPERTIES DEPENDS test_usgscsm_cam_test_save_binary_state)
set_tests_properties(test_usgscsm_cam_test_load_binary_frame PROPERTIES DEPENDS test_usgscsm_cam_test_save_binary_frame)
set_tests_properties(test_usgscsm_cam_test_load_binary_pushframe PROPERTIES DEPENDS test_usgscsm_cam_test_save_binary_pushframe)
set_tests_properties(test_usgscsm_cam_test_load_binary_sar PROPERTIES DEPENDS test_usgscsm_cam_test_save_binary_sar)
set_tests_properties(test_usgscsm_cam_test_load_binary_projected PROPERTIES DEPENDS test_usgscsm_cam_test_save_binary_projected)

# The projected sensor test fixture (constVelocityProj.json) uses a sinusoidal
# projection with Mars radius (+R=3396190) but body radii of 10 m (synthetic
# test body). Newer PROJ versions reject this mismatch. The test geometry is
# intentionally synthetic, so suppress the check.
gtest_discover_tests(runCSMCameraModelTests
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests
  PROPERTIES ENVIRONMENT "PROJ_IGNORE_CELESTIAL_BODY=YES")
