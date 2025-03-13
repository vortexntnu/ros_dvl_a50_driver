set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  ros_node.cpp
  ros_translator.cpp
)

target_include_directories(${LIB_NAME} PUBLIC
  ${CMAKE_SOURCE_DIR}/include/ros_dvl_a50_driver
)
include_directories(${CMAKE_SOURCE_DIR}/include)

target_link_libraries(${LIB_NAME} PUBLIC
  dvl_a50_driver
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2
  tf2_geometry_msgs
  vortex_msgs
)

rclcpp_components_register_node(
  ${LIB_NAME}
  PLUGIN "dvl_a50::ros::DvlA50DriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
  )

ament_export_targets(export_${LIB_NAME})

install(TARGETS ${LIB_NAME}
  EXPORT export_${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/include/
  DESTINATION include
)

install(DIRECTORY
  ${CMAKE_SOURCE_DIR}/launch
  ${CMAKE_SOURCE_DIR}/config
  DESTINATION share/${PROJECT_NAME}/
)
