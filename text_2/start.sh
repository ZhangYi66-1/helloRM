#!/bin/bash
# ------------------------------------------------------------------
# start.sh - 一键编译并运行最短路径识别程序
# 用法:
#   ./start.sh                # 编译(如有必要)并运行, 默认摄像头 0
#   ./start.sh 1              # 指定摄像头编号
#   ./start.sh paper.jpg      # 识别图片文件中的二维码
#   ./start.sh "8,8,0,1,..."  # 直接传二维码文本(调试用)
# ------------------------------------------------------------------
set -e
cd "$(dirname "$0")"

# 0. 检查源文件是否存在，避免 CMake 报出难懂的错误
if ! ls ./*.cpp >/dev/null 2>&1; then
    echo "错误: 当前目录没有 .cpp 源文件！"
    echo "请把 path_planner.cpp 拷贝到本目录:  cp /路径/path_planner.cpp ."
    exit 1
fi

BUILD_DIR="build"
BIN="$BUILD_DIR/path_planner"

# 1. 编译（二进制不存在或源码/CMakeLists 有更新时重新编译）
NEED_BUILD=0
if [ ! -f "$BIN" ]; then
    NEED_BUILD=1
elif ls -t -- *.cpp CMakeLists.txt 2>/dev/null | head -n 1 | xargs -I{} [ {} -nt "$BIN" ]; then
    NEED_BUILD=1
fi

if [ "$NEED_BUILD" -eq 1 ]; then
    echo "==> 正在编译..."
    cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BUILD_DIR" -j"$(nproc)"
    echo "==> 编译完成"
fi

# 2. 运行（所有参数原样透传给程序）
exec "$BIN" "$@"
