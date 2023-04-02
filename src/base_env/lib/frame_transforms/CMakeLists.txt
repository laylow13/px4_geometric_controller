add_library(frame_transforms STATIC frame_transforms.cpp)
target_include_directories(frame_transforms PUBLIC
    include
    ${EIGEN3_INCLUDE_DIR})
target_link_libraries(frame_transforms PRIVATE
    ${EIGEN3_LIBRARIES})
ament_target_dependencies(
    frame_transforms
    "std_msgs"
    "geometry_msgs"
    "sensor_msgs"
)