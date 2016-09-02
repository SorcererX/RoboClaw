project(RoboClaw)
cmake_minimum_required(VERSION 3.0)

set(BUILD_EXAMPLES ON CACHE BOOL "Build examples")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -g -DCV_SSE2")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O0")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto -Winline -finline-limit=2000 -mavx2 -march=native -fopenmp")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -xHOST -O3 -ipo -no-prec-div -openmp")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # N/A
endif()

add_subdirectory(examples)

aux_source_directory(src SRC_LIST)
include_directories(defs)
set( HEADER_FILES
     ${CMAKE_SOURCE_DIR}/defs/roboclaw.h )
add_library(${PROJECT_NAME} ${SRC_LIST} ${HEADER_FILES})
target_link_libraries( ${PROJECT_NAME} )
install( TARGETS ${PROJECT_NAME} DESTINATION lib )
install( FILES ${HEADER_FILES} DESTINATION include )
