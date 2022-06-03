
cmake_minimum_required(VERSION 2.8.3)
project(EDLines)


SET(BUILD_SHARED_LIBS ON)
SET(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH    ${PROJECT_SOURCE_DIR}/lib)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3")
add_definitions(-DCOMPILEDWITHC11)


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# opencv
find_package(OpenCV REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

link_directories(
    ${PROJECT_BINARY_DIR}/lib
)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
)



include_directories( include ${OpenCV_INCLUDE_DIRS} )
list(APPEND LINK_LIBS ${OpenCV_LIBS} )
file(GLOB_RECURSE all_include_files RELATIVE "${PROJECT_SOURCE_DIR}/include/" *.h *.hpp)

link_directories(${PROJECT_SOURCE_DIR}/src/)
file(GLOB_RECURSE all_source_files RELATIVE "${PROJECT_SOURCE_DIR}/src/" *.cpp )
# file(GLOB_RECURSE all_source_files *.cpp *.c)

# add_custom_target( edlines_includes DEPENDS ${all_include_files} SOURCES ${all_source_files} )


# include_directories("${PROJECT_SOURCE_DIR}/src")
# add_subdirectory(src)

# add_library( EDLine SHARED ${all_source_files})
add_library( EDLine SHARED src/edline_detector.cpp)
target_link_libraries( EDLine ${LINK_LIBS} )


add_executable(test1 test1.cpp)
target_link_libraries(test1 EDLine ${OpenCV_LIBS})

#add_library(EDLine SHARED )
#target_link_libraries(EDLine ${OpenCV_LIBS})


