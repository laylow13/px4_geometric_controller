cmake_minimum_required (VERSION 3.1)
project(FDCL-PARAM)

set (CMAKE_CXX_STANDARD 11)

set(fdcl_param_src
    src/param.cpp
)
add_library(fdcl_param STATIC ${fdcl_param_src})

target_include_directories(fdcl_param PUBLIC ${PROJECT_SOURCE_DIR}/include 
     ${PROJECT_SOURCE_DIR}/libraries/eigen)

add_executable(test_fdcl_param src/test_fdcl_param.cpp)
target_compile_options(test_fdcl_param
    PRIVATE -Wall -o -O3 -std=c++11
)
target_link_libraries(test_fdcl_param
    fdcl_param
)

