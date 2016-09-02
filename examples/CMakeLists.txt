if(BUILD_EXAMPLES)
    project( examples )
    include_directories( "${CMAKE_SOURCE_DIR}/defs" )

    add_executable( info info.cpp )
    add_dependencies( info RoboClaw )
    target_link_libraries( info RoboClaw )
endif(BUILD_EXAMPLES)
