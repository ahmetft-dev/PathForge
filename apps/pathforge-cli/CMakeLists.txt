add_executable(pathforge-cli main.cpp)
target_link_libraries(pathforge-cli PRIVATE pathforge::geometry pathforge_warnings)
