cmake_minimum_required(VERSION 3.16)
project(Class03Marker LANGUAGES CXX)

set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(class_03_demo
    main.cpp
    src/MarkerManager.cpp
)

target_include_directories(class_03_demo PRIVATE include)

target_compile_features(class_03_demo PRIVATE cxx_std_17)