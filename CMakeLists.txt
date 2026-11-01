cmake_minimum_required(VERSION 3.16)
project(RoboClaw VERSION 1.0.0 LANGUAGES CXX)

set(BUILD_EXAMPLES ON CACHE BOOL "Build examples")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++26 -O2 -g")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++26 -g -O0")

add_subdirectory(examples)

aux_source_directory(src SRC_LIST)
include_directories(defs)
set( HEADER_FILES
     ${CMAKE_SOURCE_DIR}/defs/roboclaw.h )
add_library(${PROJECT_NAME} ${SRC_LIST} ${HEADER_FILES})
target_link_libraries( ${PROJECT_NAME} )
install( TARGETS ${PROJECT_NAME} DESTINATION lib )
install( FILES ${HEADER_FILES} DESTINATION include )

# API documentation: build the "docs" target, the result is in docs/html in the build directory.
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_PROJECT_BRIEF "Packet serial driver for RoboClaw motor controllers")
  set(DOXYGEN_OUTPUT_DIRECTORY docs)
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE ${CMAKE_SOURCE_DIR}/README.md)
  set(DOXYGEN_WARN_NO_PARAMDOC YES)
  doxygen_add_docs(docs ${CMAKE_SOURCE_DIR}/README.md ${HEADER_FILES} COMMENT "Generating API documentation")
endif()

# Debian package: build the "package" target.
set(CPACK_GENERATOR DEB)
set(CPACK_PACKAGE_NAME libroboclaw-dev)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Packet serial driver for RoboClaw motor controllers")
set(CPACK_PACKAGE_CONTACT "Kai Hugo Hustoft Endresen <kai.endresen@gmail.com>")
set(CPACK_PACKAGING_INSTALL_PREFIX /usr)
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_SECTION libdevel)
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE https://github.com/SorcererX/RoboClaw)
include(CPack)
