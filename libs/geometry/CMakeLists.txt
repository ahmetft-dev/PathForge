add_library(pathforge_geometry STATIC
  src/path.cpp
  src/circle.cpp
  src/spline.cpp
  src/resample.cpp
  src/frames.cpp
  src/track.cpp
  src/shapes.cpp
)
add_library(pathforge::geometry ALIAS pathforge_geometry)
target_include_directories(pathforge_geometry PUBLIC include)
target_compile_features(pathforge_geometry PUBLIC cxx_std_20)
target_link_libraries(pathforge_geometry PRIVATE pathforge_warnings)
