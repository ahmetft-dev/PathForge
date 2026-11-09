# nlohmann/json: use an installed copy if present, otherwise a pinned, hash-checked release tarball
# (MIT licensed). Only used inside src/, never exposed in public headers.
find_package(nlohmann_json 3.11 QUIET)
if(NOT nlohmann_json_FOUND)
  include(FetchContent)
  set(JSON_BuildTests OFF CACHE INTERNAL "")
  set(JSON_Install OFF CACHE INTERNAL "")
  FetchContent_Declare(nlohmann_json
    URL https://codeload.github.com/nlohmann/json/tar.gz/refs/tags/v3.11.3
    URL_HASH SHA256=0d8ef5af7f9794e3263480193c491549b2ba6cc74bb018906202ada498a79406
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(nlohmann_json)
endif()

add_library(pathforge_io STATIC
  src/json_support.cpp
  src/track_io.cpp
  src/document_io.cpp
)
add_library(pathforge::io ALIAS pathforge_io)
target_include_directories(pathforge_io PUBLIC include)
target_compile_features(pathforge_io PUBLIC cxx_std_20)
target_link_libraries(pathforge_io
  PUBLIC pathforge::geometry pathforge::editor
  PRIVATE nlohmann_json::nlohmann_json pathforge_warnings)
