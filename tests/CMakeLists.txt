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
               StardsTests.cpp
               ../external/ale/src/Rotation.cpp
               ../external/ale/src/InterpUtils.cpp)

# StardsTests.cpp needs the star:: headers and the gate define, both of which the
# usgscsm_stards INTERFACE target carries.
if(USGSCSM_ENABLE_STARDS)
  target_link_libraries(runCSMCameraModelTests usgscsm_stards)

  # Derived from ISDs already in data/, so not committed. The .stards is converted
  # from the generated JSON state, not the ISD, so the two agree to
  # EXPECT_DOUBLE_EQ. data/frameState.stards stays committed as a format golden.
  set(STARDS_FIXTURE_DIR "${CMAKE_CURRENT_BINARY_DIR}/stards_fixtures")
  set(STARDS_FIXTURES
      "${STARDS_FIXTURE_DIR}/lineScanState.json"
      "${STARDS_FIXTURE_DIR}/lineScanState.stards"
      "${STARDS_FIXTURE_DIR}/frameState.json")
  add_custom_command(
    OUTPUT ${STARDS_FIXTURES}
    COMMAND ${CMAKE_COMMAND} -E make_directory "${STARDS_FIXTURE_DIR}"
    COMMAND csm_translate "${CMAKE_CURRENT_SOURCE_DIR}/data/toughLroNacLineScan.json"
                          "${STARDS_FIXTURE_DIR}/lineScanState.json"
    COMMAND csm_translate "${STARDS_FIXTURE_DIR}/lineScanState.json"
                          "${STARDS_FIXTURE_DIR}/lineScanState.stards"
    COMMAND csm_translate "${CMAKE_CURRENT_SOURCE_DIR}/data/simpleFramerISD.json"
                          "${STARDS_FIXTURE_DIR}/frameState.json"
    DEPENDS csm_translate
            "${CMAKE_CURRENT_SOURCE_DIR}/data/toughLroNacLineScan.json"
            "${CMAKE_CURRENT_SOURCE_DIR}/data/simpleFramerISD.json"
    COMMENT "Generating STARDS test fixtures from existing ISDs"
    VERBATIM)
  add_custom_target(stards_test_fixtures DEPENDS ${STARDS_FIXTURES})
  add_dependencies(runCSMCameraModelTests stards_test_fixtures)
  target_compile_definitions(runCSMCameraModelTests PRIVATE
                             STARDS_FIXTURE_DIR="${STARDS_FIXTURE_DIR}")
endif()
if(WIN32)
  option(CMAKE_USE_WIN32_THREADS_INIT "using WIN32 threads" ON)
  option(gtest_disable_pthreads "Disable uses of pthreads in gtest." ON)
  target_link_libraries(runCSMCameraModelTests usgscsm ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

  # Windows has no RPATH: the loader looks next to the .exe, and usgscsm.dll is built
  # one directory up. Without this the exe cannot start at all (0xc0000135), which
  # fails gtest_discover_tests' POST_BUILD probe as well as the tests themselves.
  # Registered before gtest_discover_tests so it runs first.
  add_custom_command(TARGET runCSMCameraModelTests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:usgscsm> $<TARGET_FILE_DIR:runCSMCameraModelTests>)
else()
  target_link_libraries(runCSMCameraModelTests usgscsm ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
endif()
add_executable(pluginRegistryHost PluginRegistryHost.cpp)
target_include_directories(pluginRegistryHost PRIVATE ${CSM_INCLUDE_DIR})
target_link_libraries(pluginRegistryHost ${CSM_LIBRARY} ${CMAKE_DL_LIBS})
add_test(NAME test_plugin_registers_with_host
    COMMAND pluginRegistryHost $<TARGET_FILE:usgscsm>)

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

gtest_discover_tests(runCSMCameraModelTests
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
