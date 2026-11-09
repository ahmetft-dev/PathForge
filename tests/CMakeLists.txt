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
  spline_test.cpp
  resample_test.cpp
  frames_test.cpp
  track_test.cpp
  shapes_test.cpp
  predicates_test.cpp
  width_test.cpp
  validation_test.cpp
)
target_link_libraries(pathforge_geometry_tests PRIVATE
  pathforge::geometry pathforge_warnings GTest::gtest_main)
gtest_discover_tests(pathforge_geometry_tests)

# ---- Editor (editable document) tests
add_executable(pathforge_editor_tests track_document_test.cpp derived_geometry_test.cpp)
target_link_libraries(pathforge_editor_tests PRIVATE pathforge::editor pathforge_warnings GTest::gtest_main)
gtest_discover_tests(pathforge_editor_tests)

# ---- I/O library tests
add_executable(pathforge_io_tests io_test.cpp)
target_link_libraries(pathforge_io_tests PRIVATE pathforge::io pathforge_warnings GTest::gtest_main)
target_compile_definitions(pathforge_io_tests PRIVATE PATHFORGE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
gtest_discover_tests(pathforge_io_tests)

# ---- CLI exit-code tests (exact codes are checked by cli_expect.cmake; args are joined with '|')
if(TARGET pathforge-cli)
  set(PF_DATA "${CMAKE_CURRENT_SOURCE_DIR}/data")
  function(pathforge_cli_test name expect regex)
    list(JOIN ARGN "|" joined)
    add_test(NAME cli.${name}
      COMMAND ${CMAKE_COMMAND} -DCLI=$<TARGET_FILE:pathforge-cli> -DEXPECT=${expect} "-DREGEX=${regex}"
              "-DARGS=${joined}" -P ${CMAKE_CURRENT_SOURCE_DIR}/cli_expect.cmake)
  endfunction()
  pathforge_cli_test(validate_valid 0 "valid" track validate ${PF_DATA}/valid_oval.json)
  pathforge_cli_test(validate_self_intersecting 1 "CenterlineSelfIntersection" track validate ${PF_DATA}/bowtie.json)
  pathforge_cli_test(validate_duplicate_points 1 "DegenerateSegment" track validate ${PF_DATA}/duplicate_points.json)
  pathforge_cli_test(validate_malformed 3 "malformed JSON" track validate ${PF_DATA}/malformed.json)
  pathforge_cli_test(validate_wrong_version 3 "unsupported schemaVersion 2" track validate ${PF_DATA}/wrong_version.json)
  pathforge_cli_test(validate_missing_file 3 "file not found" track validate ${PF_DATA}/does_not_exist.json)
  pathforge_cli_test(validate_usage 2 "usage" track validate)
  pathforge_cli_test(inspect_valid 0 "name +: oval" track inspect ${PF_DATA}/valid_oval.json)
  pathforge_cli_test(inspect_invalid_still_exit_0 0 "invalid" track inspect ${PF_DATA}/bowtie.json)
  pathforge_cli_test(circle_report 0 "analytical length" circle --radius 100 --points 360)
  pathforge_cli_test(oval_report 0 "oval" oval --length 300 --width 120 --points 512)
  pathforge_cli_test(scurve_report 0 "sign changes +: 1" scurve --points 256)
  pathforge_cli_test(bad_parameter 2 "stadium length must be" oval --length 100 --width 120)
  add_test(NAME cli.export_roundtrip
    COMMAND ${CMAKE_COMMAND} -DCLI=$<TARGET_FILE:pathforge-cli> -DOUT=${CMAKE_CURRENT_BINARY_DIR}/cli_roundtrip.json
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cli_roundtrip.cmake)
endif()
