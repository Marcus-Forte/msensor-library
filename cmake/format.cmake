# Format
find_program(formatter clang-format)
if(formatter)
  file(GLOB_RECURSE sourcefiles
    "include/**/*.hh"
    "src/**/*.cc"
    "src/*.cc"
    "grpc/*.cc"
    "grpc/*.hh")
  string (REPLACE ";" " " sourcefiles "${sourcefiles}")
  add_custom_target(format ALL
  COMMAND sh -c "clang-format -i ${sourcefiles}"
  VERBATIM)
endif()