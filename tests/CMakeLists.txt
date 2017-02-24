# Find libcsmapi.so
FIND_LIBRARY(CSMAPI_LIBRARY csmapi "${CMAKE_SOURCE_DIR}/lib")

# Find the unit tests
FILE(GLOB SRC_FILES ${CMAKE_SOURCE_DIR}/tests/ *Test.cpp)

ADD_EXECUTABLE(runTests runTests.cpp ${SRC_FILES} ${HEADER_FILES})
TARGET_INCLUDE_DIRECTORIES(runTests PUBLIC
  ${CMAKE_SOURCE_DIR}/include/mdis
  ${CMAKE_SOURCE_DIR}/include/

TARGET_LINK_LIBRARIES(runTests gtest_main
                      MdisPlugin
                      MdisNacSensorModel
                      ${CSMAPI_LIBRARY})
