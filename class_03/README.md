## 1. 文件职责说明
- `include/MarkerManager.hpp`：头文件。包含 `Marker` 结构体的定义，以及 `MarkerManager` 类的声明（包括 `add`、`number` 等成员函数的声明）。
- `src/MarkerManager.cpp`：源文件。实现 `MarkerManager` 类中声明的各个成员函数的具体逻辑。
- `main.cpp`：主程序。负责创建初始记录数据，调用统计函数，并演示指针和引用的使用方法。
- `CMakeLists.txt`：构建配置文件。定义项目名称、C++标准，以及如何将源文件编译链接为可执行程序。
## 2. AI使用说明
是否使用 AI：是。

用途：辅助理解 C++ 的 std::vector 和 std::map 的底层逻辑；排查 CMakeLists.txt 的语法错误；理清头文件与源文件分离时遇到的“重复定义”与“未定义引用”的报错原因。

采纳内容：参考了 AI 提供的 CMake 现代写法（target_include_directories），以及修正结构体中未初始化变量 total 的方法。

核对方法：AI 给出的代码逻辑经过我仔细阅读后，我会确保它符合题目要求（例如区分了业务编号 id 与容器下标），并在本地终端执行 cmake --build build 进行编译验证，确认输出结果与预期一致后才予以采纳。

## 3. 实际使用的运行命令：
mkdir...
touch...
code...
cd...
cmake -S . -B build
cmake --build build
./build/class_03_demo 