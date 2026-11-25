cmake_minimum_required(VERSION 3.16)
project(text LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV
find_package(OpenCV 4 REQUIRED)

# 生成可执行文件，名字必须叫 text（符合截图里的 ./text）
add_executable(text main.cpp)

# 包含 OpenCV 头文件路径
target_include_directories(text PRIVATE ${OpenCV_INCLUDE_DIRS})

# 链接 OpenCV 库
target_link_libraries(text PRIVATE ${OpenCV_LIBS})