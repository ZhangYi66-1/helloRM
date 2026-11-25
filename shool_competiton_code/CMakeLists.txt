cmake_minimum_required(VERSION 3.10)
project(school_competition_code)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

# ---- zbar（可选但强烈建议）----
# 本机 OpenCV 未链接 QUIRC，QRCodeDetector 只能检测、不能解码；
# 必须启用 zbar 才能真正解出二维码内容（与原版 path_planner 一致）。
find_library(ZBAR_LIB NAMES zbar)
if(ZBAR_LIB)
    add_compile_definitions(HAVE_ZBAR)
    message(STATUS "zbar: ${ZBAR_LIB} -> 已启用 zbar 解码 (HAVE_ZBAR)")
else()
    message(WARNING "未找到 libzbar！QRCodeDetector 无 QUIRC 将解不出任何二维码。"
                    "请执行: sudo apt install libzbar-dev")
endif()


add_executable(school_competition_code main.cpp)
target_include_directories(school_competition_code PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(school_competition_code ${OpenCV_LIBS})
if(ZBAR_LIB)
    target_link_libraries(school_competition_code ${ZBAR_LIB})
endif()
