add_library(pathforge_editor STATIC
  src/track_document.cpp
)
add_library(pathforge::editor ALIAS pathforge_editor)
target_include_directories(pathforge_editor PUBLIC include)
target_compile_features(pathforge_editor PUBLIC cxx_std_20)
target_link_libraries(pathforge_editor PUBLIC pathforge::geometry PRIVATE pathforge_warnings)
