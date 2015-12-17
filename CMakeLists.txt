cmake_minimum_required(VERSION 2.6)
set(CMAKE_CXX_COMPILER g++)

project(kdtree C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall  -std=gnu++0x -fpermissive")
include_directories( ${PROJECT_SOURCE_DIR}/include/ )

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)
############################

set(EXAMPLE_SOURCES ${PROJECT_SOURCE_DIR}/examples/)

file( GLOB EXECUTABLE_SOURCES  RELATIVE ${EXAMPLE_SOURCES} ${EXAMPLE_SOURCES}*.cpp)

foreach( executablesourcefile ${EXECUTABLE_SOURCES} )

    string( REPLACE ".cpp" "" local_executable ${executablesourcefile})
    MESSAGE("-- Adding executable ${local_executable} from code ${executablesourcefile}")
    add_executable(${local_executable}  ${EXAMPLE_SOURCES}${executablesourcefile})
    target_link_libraries( ${local_executable} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY}) # this line is juts needed if  te executable need to link any library
endforeach( executablesourcefile ${EXECUTABLE_SOURCES} )


find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.in @ONLY)
	add_custom_target(doc
	${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/doxygen.in
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Generating documentation with Doxygen" VERBATIM)
endif(DOXYGEN_FOUND)