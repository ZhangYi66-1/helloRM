#!/usr/bin/env bash
# ============================================================================
#  ./start.sh  —— 一键编译并运行自动迫击炮全流程
# ----------------------------------------------------------------------------
#  用法：
#      ./start.sh              编译 + 运行（串口默认 /dev/ttyUSB0）
#      ./start.sh /dev/ttyUSB1 指定串口
#  首次使用前（如果提示 Permission denied）：
#      chmod +x start.sh
# ============================================================================

set -e

# 切到脚本所在目录，保证 result/ 生成在工程根目录（不是 build/ 里）
cd "$(dirname "$0")"

echo "==> [1/2] 配置 + 编译"
cmake -S . -B build
cmake --build build -j 2

echo "==> [2/2] 运行"
exec ./build/school_competition_code "$@"
