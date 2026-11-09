set(CMAKE_AUTOMOC ON)

add_executable(pathforge-studio
  main.cpp
  MainWindow.cpp
  MainWindow.h
  TrackViewport.cpp
  TrackViewport.h
)
target_link_libraries(pathforge-studio PRIVATE
  pathforge::io Qt6::Widgets pathforge_warnings)
set_target_properties(pathforge-studio PROPERTIES
  WIN32_EXECUTABLE ON
  MACOSX_BUNDLE ON)
