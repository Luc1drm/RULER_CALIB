cmake_minimum_required(VERSION 3.10)

project(sensors_calib)
set (CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(LIBRARY_NAME ${PROJECT_NAME})

set(RAPIDJSON_INCLUDE_DIRS "D:/Program Files/rapidjson/include")
include_directories(${RAPIDJSON_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
set(PCL_DIR " D:\\Program Files\\PCL\ 1.13.0")
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_compile_options(
  "$<$<CONFIG:DEBUG>:-DDEBUG>"
)

add_subdirectory(src)
add_subdirectory(examples)