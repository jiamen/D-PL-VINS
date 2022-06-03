cmake_minimum_required(VERSION 2.8.3)
project(d-pl-vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_model
    #eigen_stl_containers
    )



# Set(OpenCV_DIR "/home/zlc/Tools/SLAMTools/opencv-3.4.17/build")
# find_package(OpenCV 3.4.17 REQUIRED)
Set(OpenCV_DIR "/home/zlc/Tools/SLAMTools/opencv-3.3.1/build")
find_package(OpenCV 3.3.1 REQUIRED)
# find_package(OpenCV REQUIRED)


find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

LIST(APPEND SOURCEFILES
        src/parameters.cpp
        src/feature_manager.cpp
        src/feature_tracker/feature_tracker.cpp

        src/factor/pose_local_parameterization.cpp
        src/factor/projection_factor.cpp
        src/factor/marginalization_factor.cpp

        src/utility/utility.cpp
        src/utility/visualization.cpp
        src/utility/CameraPoseVisualization.cpp
        src/utility/line_geometry.h
        src/utility/line_geometry.cpp

        src/initial/solve_5pts.cpp
        src/initial/initial_aligment.cpp
        src/initial/initial_sfm.cpp
        src/initial/initial_ex_rotation.cpp

        src/loop-closure/ThirdParty/DBoW/BowVector.cpp
        src/loop-closure/ThirdParty/DBoW/FBrief.cpp
        src/loop-closure/ThirdParty/DBoW/FeatureVector.cpp
        src/loop-closure/ThirdParty/DBoW/QueryResults.cpp
        src/loop-closure/ThirdParty/DBoW/ScoringObject.cpp
        src/loop-closure/ThirdParty/DUtils/Random.cpp
        src/loop-closure/ThirdParty/DUtils/Timestamp.cpp
        src/loop-closure/ThirdParty/DVision/BRIEF.cpp
        src/loop-closure/ThirdParty/VocabularyBinary.cpp
        src/loop-closure/loop_closure.cpp
        src/loop-closure/keyframe.cpp
        src/loop-closure/keyframe_database.cpp

        src/sophus/common.hpp
        src/sophus/types.hpp
        src/sophus/so3.hpp
        src/sophus/se3.h
        src/sophus/so3.h

        src/factor/line_parameterization.h
        src/factor/line_parameterization.cpp
        src/factor/line_projection_factor.h
        src/factor/line_projection_factor.cpp

        src/plane_detection/plane_detection.cpp

        src/factor/relativepose_factor.cpp
        src/factor/projection_plane_factor.cpp

)
ADD_LIBRARY(vinsEstimatorLib SHARED ${SOURCEFILES})
# TARGET_LINK_LIBRARIES(vinsEstimatorLib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
TARGET_LINK_LIBRARIES(vinsEstimatorLib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} /home/zlc/Tools/Sophus/build/libSophus.so)


add_executable(d-pl-vins_estimator
    src/estimator_node.cpp
    src/estimator.cpp
    )
target_link_libraries(d-pl-vins_estimator vinsEstimatorLib)

