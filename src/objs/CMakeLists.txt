INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/mdis")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/")

ADD_LIBRARY(MdisNacSensorModel SHARED MdisNacSensorModel.cpp)
ADD_LIBRARY(MdisPlugin SHARED MdisPlugin.cpp)
