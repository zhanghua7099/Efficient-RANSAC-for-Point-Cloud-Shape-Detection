# Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/CMakeLists.txt
cmake_minimum_required(VERSION 2.8)
project(pc_ransac)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

include_directories(./)
include_directories(./MiscLib/)
file(GLOB_RECURSE cpp_file "src/*.cpp")
file(GLOB_RECURSE MISC_SRC "MiscLib/*.cpp")

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
)

add_library(libransac STATIC  ${cpp_file} ${MISC_SRC})
add_executable(main main.cpp)
target_link_libraries(main libransac)
