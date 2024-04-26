cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#add_subdirectory(lib/fdcl_param)
#add_subdirectory(lib/fdcl_se3_controller)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(base_env REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories( 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/fdcl_param/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/fdcl_se3_controller/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

add_executable(se3_geometric_controller 
  src/controller_node.cpp lib/se3_controller/controller.cpp)


# add_executable(fm_controller src/fm_control.cpp)

# target_link_libraries(fm_controller  fdcl_se3_controller)

target_compile_features(se3_geometric_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# target_compile_features(fm_controller PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  se3_geometric_controller
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "px4_msgs"
  "base_env"
)

# ament_target_dependencies(
#   fm_controller
#   "rclcpp"
#   "std_msgs"
#   "geometry_msgs"
#   "px4_msgs"
#   "base_env"
# )



add_executable(disturbance_observer 
  src/dob_node.cpp)

target_include_directories(disturbance_observer PRIVATE lib/lpf)

target_compile_features(disturbance_observer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  disturbance_observer
  "rclcpp"
  "base_env"
)

install(TARGETS se3_geometric_controller disturbance_observer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
