# Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/CMakeLists.txt
cmake_minimum_required(VERSION 3.18)
project(pc_ransac LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include_directories(./)
include_directories(./MiscLib/)
file(GLOB_RECURSE cpp_file "src/*.cpp")
file(GLOB_RECURSE MISC_SRC "MiscLib/*.cpp")

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
)

add_library(libransac STATIC  ${cpp_file} ${MISC_SRC})
target_compile_features(libransac PUBLIC cxx_std_17)
set_target_properties(libransac PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(libransac PUBLIC -fno-strict-aliasing)
endif()

add_executable(main main.cpp)
target_compile_features(main PRIVATE cxx_std_17)
target_link_libraries(main libransac)

option(PC_RANSAC_BUILD_PYTHON "Build Python bindings with nanobind" OFF)

if(PC_RANSAC_BUILD_PYTHON)
    find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

    execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_VARIABLE nanobind_ROOT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(nanobind_ROOT)
        list(APPEND CMAKE_PREFIX_PATH "${nanobind_ROOT}")
    endif()

    find_package(nanobind CONFIG REQUIRED)
    if(DEFINED nanobind_VERSION AND nanobind_VERSION VERSION_GREATER_EQUAL "2.10")
        message(FATAL_ERROR
            "nanobind ${nanobind_VERSION} does not support Python 3.8. "
            "Install nanobind >=2.0,<2.10 for Python 3.8 builds.")
    endif()

    nanobind_add_module(pc_ransac python/bindings.cpp)
    target_link_libraries(pc_ransac PRIVATE libransac)
    target_include_directories(pc_ransac PRIVATE
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/MiscLib
    )

    install(TARGETS pc_ransac LIBRARY DESTINATION .)
endif()
