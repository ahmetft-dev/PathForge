find_package(GTest QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_Declare(googletest
    URL https://codeload.github.com/google/googletest/tar.gz/refs/tags/v1.17.0
    URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(googletest)
endif()

include(GoogleTest)

add_executable(pathforge_geometry_tests
  vec2_test.cpp
  path_test.cpp
  circle_test.cpp
)
target_link_libraries(pathforge_geometry_tests PRIVATE
  pathforge::geometry pathforge_warnings GTest::gtest_main)
gtest_discover_tests(pathforge_geometry_tests)
