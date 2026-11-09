set(CMAKE_AUTOMOC ON)

# Everything except main() lives in a library so that the Qt tests can drive the real widgets.
add_library(pathforge_studio_lib STATIC
  EditorSession.cpp
  EditorSession.h
  MainWindow.cpp
  MainWindow.h
  TrackViewport.cpp
  TrackViewport.h
  ValidationRunner.cpp
  ValidationRunner.h
)
target_include_directories(pathforge_studio_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pathforge_studio_lib
  PUBLIC pathforge::io Qt6::Widgets
  PRIVATE pathforge_warnings)

add_executable(pathforge-studio main.cpp)
target_link_libraries(pathforge-studio PRIVATE pathforge_studio_lib pathforge_warnings)
set_target_properties(pathforge-studio PROPERTIES
  WIN32_EXECUTABLE ON
  MACOSX_BUNDLE ON)

# Qt widget tests (QtTest, offscreen platform). Built only when Qt's Test module is available.
if(PATHFORGE_BUILD_TESTS)
  find_package(Qt6 QUIET COMPONENTS Test)
  if(Qt6Test_FOUND)
    add_executable(pathforge_studio_tests ${PROJECT_SOURCE_DIR}/tests/studio_test.cpp)
    target_link_libraries(pathforge_studio_tests PRIVATE pathforge_studio_lib Qt6::Test pathforge_warnings)
    target_compile_definitions(pathforge_studio_tests PRIVATE PATHFORGE_TEST_DATA_DIR="${PROJECT_SOURCE_DIR}/tests/data")
    add_test(NAME studio.widgets COMMAND pathforge_studio_tests)
    set_tests_properties(studio.widgets PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")

    add_executable(pathforge_studio_async_tests ${PROJECT_SOURCE_DIR}/tests/studio_async_test.cpp)
    target_link_libraries(pathforge_studio_async_tests PRIVATE pathforge_studio_lib Qt6::Test pathforge_warnings)
    add_test(NAME studio.async_validation COMMAND pathforge_studio_async_tests)
    set_tests_properties(studio.async_validation PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  else()
    message(STATUS "Qt6 Test not found: skipping Studio widget tests")
  endif()
endif()
