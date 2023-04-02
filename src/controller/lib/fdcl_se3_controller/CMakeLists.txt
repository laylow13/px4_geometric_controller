cmake_minimum_required(VERSION 3.8)

project(fdcl_se3)

find_package(Eigen3 REQUIRED)

add_library(fdcl_se3_controller STATIC fdcl_control.cpp 
  fdcl_matrix_utils.cpp)

target_include_directories(fdcl_se3_controller PUBLIC   
    include 
    PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    )

target_link_libraries(fdcl_se3_controller PRIVATE 
    fdcl_param ${EIGEN3_LIBRARY})