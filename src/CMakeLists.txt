add_executable(ros_dvl_a50_driver_node main.cpp
    ros_node.cpp
    ros_translator.cpp
)

target_include_directories(ros_dvl_a50_driver_node PUBLIC
    ${CMAKE_SOURCE_DIR}/include/ros_dvl_a50_driver
)
include_directories(${CMAKE_SOURCE_DIR}/include)

target_link_libraries(ros_dvl_a50_driver_node
    dvl_a50_driver
)

ament_target_dependencies(ros_dvl_a50_driver_node
    rclcpp
    geometry_msgs
    tf2
    tf2_geometry_msgs
    vortex_msgs
)

install(TARGETS ros_dvl_a50_driver_node
    DESTINATION lib/${PROJECT_NAME}
)

