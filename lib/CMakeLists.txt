add_library(dvl_a50_driver STATIC src/driver.cpp)

target_include_directories(dvl_a50_driver PUBLIC
    include
)

target_compile_options(dvl_a50_driver PUBLIC -fPIC)

target_link_libraries(dvl_a50_driver PUBLIC
    Boost::boost
    nlohmann_json::nlohmann_json
    spdlog::spdlog
)

install(TARGETS dvl_a50_driver
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
