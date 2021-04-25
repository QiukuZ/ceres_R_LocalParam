cmake_minimum_required( VERSION 2.8 )
project( OptimProblem )

add_definitions(-std=c++14)

SET(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED )
find_package(Ceres REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable( ceres_demo_R ceres_demo_R.cpp)
target_link_libraries( ceres_demo_R ${CERES_LIBRARIES})

