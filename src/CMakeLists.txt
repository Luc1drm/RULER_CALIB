cmake_minimum_required(VERSION 3.10)

set(SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/CalibrationHandler.cpp
  ${PROJECT_SOURCE_DIR}/src/HistogramHandler.cpp
  ${PROJECT_SOURCE_DIR}/src/ParamConfig.cpp
  ${PROJECT_SOURCE_DIR}/src/ProbabilityHandler.cpp
  ${PROJECT_SOURCE_DIR}/src/Types.cpp
  ${PROJECT_SOURCE_DIR}/src/Utility.cpp
)

add_library(${LIBRARY_NAME}
  STATIC
    ${SOURCE_FILES}
)

target_include_directories(${LIBRARY_NAME}
  SYSTEM PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${RAPIDJSON_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_NAME}
  PUBLIC
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Platform-specific compile options
target_compile_options(${LIBRARY_NAME}
  PRIVATE
    $<$<AND:$<CONFIG:Debug>,$<CXX_COMPILER_ID:MSVC>>:/Od /DEBUG /W4 /WX>
    $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:MSVC>>:/O2 /W4 /WX>
    $<$<AND:$<CONFIG:Debug>,$<NOT:$<CXX_COMPILER_ID:MSVC>>>:-O0 -g -Wall -Werror>
    $<$<AND:$<CONFIG:Release>,$<NOT:$<CXX_COMPILER_ID:MSVC>>>:-O3 -Wall -Werror>
)
