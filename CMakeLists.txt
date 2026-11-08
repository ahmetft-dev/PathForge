cmake_minimum_required(VERSION 3.21)
project(PathForge VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(PATHFORGE_BUILD_TESTS "Build unit tests (fetches GoogleTest if not installed)" ON)
option(PATHFORGE_BUILD_CLI "Build pathforge-cli" ON)
# AUTO: build Studio only if Qt 6 is found. ON: require Qt. OFF: skip.
set(PATHFORGE_BUILD_STUDIO AUTO CACHE STRING "Build PathForge Studio (AUTO, ON, OFF)")
set_property(CACHE PATHFORGE_BUILD_STUDIO PROPERTY STRINGS AUTO ON OFF)
option(PATHFORGE_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

# Warning flags are applied per target through this interface library.
add_library(pathforge_warnings INTERFACE)
if(MSVC)
  target_compile_options(pathforge_warnings INTERFACE /W4 /permissive-)
  if(PATHFORGE_WARNINGS_AS_ERRORS)
    target_compile_options(pathforge_warnings INTERFACE /WX)
  endif()
else()
  target_compile_options(pathforge_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
  if(PATHFORGE_WARNINGS_AS_ERRORS)
    target_compile_options(pathforge_warnings INTERFACE -Werror)
  endif()
endif()
