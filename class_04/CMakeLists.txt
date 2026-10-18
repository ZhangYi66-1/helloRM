cmake_minimum_required(VERSION 3.16)
project(class_04 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV
find_package(OpenCV 4 REQUIRED)

# 生成可执行文件，名字必须叫 class_04（符合截图里的 ./class_04）
add_executable(class_04 main.cpp)

# 包含 OpenCV 头文件路径
target_include_directories(class_04 PRIVATE ${OpenCV_INCLUDE_DIRS})

# 链接 OpenCV 库
target_link_libraries(class_04 PRIVATE ${OpenCV_LIBS})

