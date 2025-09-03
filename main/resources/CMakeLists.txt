cmake_minimum_required(VERSION 3.10)

set(ts_examples_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../ts-examples)
set(ts_examples_tgz ts_examples.tar.gz)

file(GLOB_RECURSE ts_examples_files ${ts_examples_dir}/*)

# generate tar file
add_custom_command(
    OUTPUT ${ts_examples_tgz}
    DEPENDS ${ts_examples_files}
    COMMAND tar -czf ${ts_examples_tgz} -C ${ts_examples_dir} .
)

# copy resources into resources.cpp
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/resources.cpp
    DEPENDS ${ts_examples_tgz} resources.cpp.in resources.cmake
    COMMAND "${CMAKE_COMMAND}"
        -D "resources_dir=${CMAKE_CURRENT_BINARY_DIR}"
        -D "src_in=${CMAKE_CURRENT_SOURCE_DIR}/resources.cpp.in"
        -D "src_out=${CMAKE_CURRENT_BINARY_DIR}/resources.cpp"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/resources.cmake"
)

add_library(resources STATIC ${CMAKE_CURRENT_BINARY_DIR}/resources.cpp)
