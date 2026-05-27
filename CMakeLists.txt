# Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(pc_ransac LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

include_directories(./)
include_directories(./MiscLib/)
file(GLOB_RECURSE cpp_file "src/*.cpp")
file(GLOB_RECURSE MISC_SRC "MiscLib/*.cpp")

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
)

add_library(libransac STATIC  ${cpp_file} ${MISC_SRC})
target_compile_features(libransac PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(libransac PUBLIC -fno-strict-aliasing)
endif()

add_executable(main main.cpp)
target_compile_features(main PRIVATE cxx_std_17)
target_link_libraries(main libransac)
