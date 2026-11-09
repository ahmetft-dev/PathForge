add_executable(pathforge-cli main.cpp)
target_link_libraries(pathforge-cli PRIVATE pathforge::io pathforge_warnings)  # geometry comes in transitively
