cmake_minimum_required(VERSION 3.8)
project(base_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# find_package(rospy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_subdirectory(lib/frame_transforms)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UAVMotionLinear.msg"
  "msg/UAVMotionAngular.msg"
  "msg/UAVMotion.msg"
  "msg/UAVState.msg"
  "msg/UAVCmd.msg"
  "msg/UAVThrust.msg"
  "msg/UAVDisturbance.msg"
  DEPENDENCIES geometry_msgs std_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(mocap_data_convert src/mocap_data_convert.cpp)

target_include_directories(mocap_data_convert PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

target_compile_features(mocap_data_convert PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

ament_target_dependencies(
  mocap_data_convert
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
)

add_executable(sim_base src/sim_base.cpp)

target_include_directories(sim_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/frame_transforms/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(sim_base frame_transforms)

target_compile_features(sim_base PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

ament_target_dependencies(
  sim_base
  "rclcpp"
  "px4_msgs"
)

add_executable(trajectory_generator src/trajectory_generator.cpp)

target_include_directories(trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/frame_transforms/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

target_compile_features(trajectory_generator PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

ament_target_dependencies(
  trajectory_generator
  "rclcpp"
)

add_executable(external_pose_to_fcu src/external_pose_to_fcu.cpp)

target_include_directories(external_pose_to_fcu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

target_compile_features(external_pose_to_fcu PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

ament_target_dependencies(
  external_pose_to_fcu
  "rclcpp"
  "nav_msgs"
  "px4_msgs"
)

add_executable(mocap_feedback_test src/mocap_feedback_TEST.cpp)

target_include_directories(mocap_feedback_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

target_compile_features(mocap_feedback_test PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

ament_target_dependencies(
  mocap_feedback_test
  "rclcpp"
  "px4_msgs"
)

rosidl_target_interfaces(mocap_data_convert
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(sim_base
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(trajectory_generator
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS mocap_data_convert external_pose_to_fcu sim_base trajectory_generator mocap_feedback_test
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
