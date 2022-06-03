cmake_minimum_required(VERSION 3.6)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


#Set(OpenCV_DIR "/home/zlc/Tools/SLAMTools/opencv-3.4.17/build")
#find_package(OpenCV 3.4.17 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    message_filters
    )

# Set(OpenCV_DIR "/home/healer/Downloads/opencv3.2/opencv-3.2.0/build") # important find opencv version   这里是zlc去除的
# find_package(OpenCV 3.2 QUIET)




MESSAGE(STATUS "OpenCV version: "${OpenCV_VERSION})
MESSAGE(${PROJECT_SOURCE_DIR})
MESSAGE(${PROJECT_BINARY_DIR})
message("opencv include dir: " ${OpenCV_INCLUDE_DIRS})
message("opencv config path: " ${OpenCV_CONFIG_PATH})





catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    /home/zlc/cpl-tmp_ws/src/PL-VINS/feature_tracker/src/line_descriptor/include # find LSD
) 

# add_executable(feature_tracker
#    src/feature_tracker_node.cpp
#    src/parameters.cpp
#    src/feature_tracker.cpp
#    )
# target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})




# line detect and match
add_executable(LineFeature_tracker    
    src/linefeature_tracker_node.cpp
    src/parameters.cpp
    src/linefeature_tracker.cpp
    src/linefeature_tracker.h
    src/tic_toc.h

    src/line_descriptor/src/binary_descriptor_custom.cpp
    src/line_descriptor/src/binary_descriptor_matcher.cpp
    src/line_descriptor/src/bitarray_custom.hpp
    src/line_descriptor/src/bitops_custom.hpp
    src/line_descriptor/src/draw_custom.cpp
    src/line_descriptor/src/LSDDetector_custom.cpp
    src/line_descriptor/src/precomp_custom.hpp
    src/line_descriptor/src/types_custom.hpp


    src/EDLines/src/edline_detector.cpp
    src/EDLines/include/edline_detector.h
    src/EDLines/include/line.h
)

# /home/healer/PL-VINS/PLVINS/PL-VINS/feature_tracker
target_link_libraries(LineFeature_tracker 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    #/home/zlc/cpl-tmp_ws/src/PL-VINS/feature_tracker/src/line_descriptor/lib/liblinedesc.so  # line feature
    #/home/zlc/cpl-tmp_ws/src/PL-VINS/feature_tracker/src/EDLines/lib/libEDLine.so
)
