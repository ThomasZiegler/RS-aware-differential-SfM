cmake_minimum_required(VERSION 2.8)

project(diff_RS_SfM)


add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system thread filesystem REQUIRED REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${GFLAGS_LIBRARIES})
include_directories(${OpenCV_INLCUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(diff_RS_SfM main.cc camera.cc camera.h rsframe.cc rsframe.h scanline.cc scanline.h
        nonlinearRefinement.cc nonlinearRefinement.h minimal.cc minimal.h errorMeasure.cpp errorMeasure.h)
target_link_libraries(diff_RS_SfM ${CERES_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})


