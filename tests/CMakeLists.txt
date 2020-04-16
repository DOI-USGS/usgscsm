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

gtest_discover_tests(runCSMCameraModelTests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
