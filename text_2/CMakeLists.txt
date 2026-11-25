cmake_minimum_required(VERSION 3.10)
project(path_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 链接系统找到的全部 OpenCV 模块
# 注意: QRCodeDetector 位于 opencv_objdetect 模块，必须链接，
# 否则编译通过但链接时报 undefined reference to cv::QRCodeDetector::...
find_package(OpenCV REQUIRED)

# 自动收集当前目录下所有 .cpp 作为源文件（不再依赖固定文件名）
file(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
if(SOURCES STREQUAL "")
    message(FATAL_ERROR
        "当前目录没有找到任何 .cpp 源文件！请把 path_planner.cpp 放到 CMakeLists.txt 同一目录下。")
endif()
message(STATUS "源文件: ${SOURCES}")

add_executable(path_planner ${SOURCES})

target_include_directories(path_planner PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(path_planner PRIVATE ${OpenCV_LIBS})
target_compile_options(path_planner PRIVATE -O2 -Wall)

# 可选: 自动检测 zbar（更强的二维码解码器）。安装: sudo apt install libzbar-dev
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(ZBAR QUIET zbar)
endif()
if(ZBAR_FOUND)
    target_compile_definitions(path_planner PRIVATE HAVE_ZBAR)
    target_include_directories(path_planner PRIVATE ${ZBAR_INCLUDE_DIRS})
    target_link_libraries(path_planner PRIVATE ${ZBAR_LIBRARIES})
    message(STATUS "已启用 zbar 二维码解码兜底")
else()
    message(STATUS "未检测到 zbar，仅使用 OpenCV 解码（可 sudo apt install libzbar-dev 增强）")
endif()