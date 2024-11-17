add_library(dvl_a50_driver STATIC parser.cpp tcp_client.cpp)

target_include_directories(dvl_a50_driver PUBLIC
    ${CMAKE_SOURCE_DIR}/include/ros_dvl_a50_driver
)

target_link_libraries(dvl_a50_driver PUBLIC
    Boost::boost
    nlohmann_json::nlohmann_json
)

install(TARGETS dvl_a50_driver
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ros_dvl_a50_driver/
    DESTINATION include/ros_dvl_a50_driver
)
