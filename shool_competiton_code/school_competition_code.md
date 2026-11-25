# C++ / 串口 / OpenCV 语法手册

> 基于你的项目 `school_competition_code`（`main.cpp`、`CMakeLists.txt`、`SimpleSerial` 串口类）整理。
> 每一项都按你要的三点写：**① 语法格式 / 参数及类型　② 用法　③ 项目中的应用场景举例**。
> 建议配合代码看：打开 `main.cpp`，对照章节里的函数名找过去。

---

## 目录

1. [程序骨架与预处理](#1-程序骨架与预处理)
2. [变量、常量与基本类型](#2-变量常量与基本类型)
3. [运算符与表达式](#3-运算符与表达式)
4. [控制流](#4-控制流)
5. [函数](#5-函数)
6. [数组、vector 与 string](#6-数组vector-与-string)
7. [结构体、类与运算符重载](#7-结构体类与运算符重载)
8. [指针、引用与内存模型](#8-指针引用与内存模型)
9. [Lambda 表达式](#9-lambda-表达式)
10. [标准库容器与算法](#10-标准库容器与算法)
11. [输入输出与格式化](#11-输入输出与格式化)
12. [时间与线程](#12-时间与线程)
13. [POSIX 串口编程](#13-posix-串口编程)
14. [OpenCV 核心类型](#14-opencv-核心类型)
15. [摄像头与窗口/文件](#15-摄像头与窗口文件)
16. [图像预处理函数](#16-图像预处理函数)
17. [几何变换与透视](#17-几何变换与透视)
18. [轮廓处理](#18-轮廓处理)
19. [绘图与文字](#19-绘图与文字)
20. [二维码专用 API](#20-二维码专用-api)
21. [综合案例：detectShapes 逐行拆解](#21-综合案例detectshapes-逐行拆解)
22. [常见坑与学习建议](#22-常见坑与学习建议)

---

## 1. 程序骨架与预处理

### 1.1 `#include` 头文件

- **格式**：`#include <标准库头>` 或 `#include <opencv2/xxx.hpp>`
- **用法**：在编译前把另一个文件的声明"复制"进来。`<>` 查系统/第三方目录，`""` 先查当前目录。
- **例子**：
  ```cpp
  #include <opencv2/opencv.hpp>   // OpenCV 全家桶（Mat、摄像头、图像处理、绘图）
  #include <opencv2/imgproc.hpp>  // 图像处理（findContours 等），其实已被上面包含
  #include <zbar.h>               // 二维码解码库（条件包含）
  #include <iostream>             // cout / cerr
  #include <vector>               // std::vector
  #include <termios.h>            // POSIX 串口配置
  ```

### 1.2 命名空间 `using namespace`

- **格式**：`using namespace std;` / `using namespace cv;`
- **用法**：省掉前缀。`std::cout` → `cout`，`cv::Mat` → `Mat`。**只建议在 .cpp 里用，不要写在头文件里**。
- **例子**：项目里 `cout << "[串口] ..."`、`Mat gray;` 能直接写，靠的就是这两行。

### 1.3 条件编译 `#ifdef` / `#if` / `#endif`

- **格式**：
  ```cpp
  #ifdef 宏名
      // 宏存在才编译
  #else
      // 否则编译这里
  #endif
  ```
- **用法**：同一份源码在不同环境下编出不同结果（典型：可选依赖、版本兼容）。
- **例子 1**：zbar 是可选的，没装就不编译那段：
  ```cpp
  #ifdef HAVE_ZBAR
  static string zbarDecode(const Mat& gray) { ... }
  #endif
  ```
  `HAVE_ZBAR` 由 `CMakeLists.txt` 里的 `add_compile_definitions(HAVE_ZBAR)` 提供。
- **例子 2**：按 OpenCV 版本决定用不用更高级的 API：
  ```cpp
  #if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR > 5) ...
      qr.detectAndDecodeCurved(...);
  #endif
  ```

### 1.4 `static` 修饰函数和全局常量

- **格式**：`static 返回类型 函数名(...)`；`static const 类型 名字 = 值;`
- **用法**：
  - 修饰**函数**：限定只在本 `.cpp` 内可见（内部链接），不会和别的文件重名冲突。项目里所有工具函数都是 `static`。
  - 修饰**全局常量**：文件内私有，编译期常量。
  - 修饰**函数内局部变量**：只初始化一次，之后跨调用保持（见 `openKernel`）。
- **例子**：
  ```cpp
  static const int SERIAL_BAUD = 9600;              // 全局静态常量
  static string tryDecode(const Mat& img, QRCodeDetector& qr) { ... }  // 文件内函数
  static const Mat openKernel = getStructuringElement(...); // 只构造一次
  ```

---

## 2. 变量、常量与基本类型

### 2.1 基本类型一览（项目里用到的）

| 类型 | 含义 | 项目中的例子 |
|---|---|---|
| `int` | 32 位整数 | `TURN_LEFT_LONG_MS = 1500` |
| `double` | 双精度浮点 | `double s = 640.0 / frame.cols;` |
| `float` | 单精度浮点 | `Point2f baseSide`、`(float)margin` |
| `bool` | true/false | `bool sameColor`、`bool detected` |
| `char` | 单字符 | `char ts[64]` |
| `size_t` | 无符号整数（容器下标类型） | `for (size_t i = 0; i < contours.size(); ++i)` |
| `int64_t` | 64 位整数 | `int64_t t0 = nowMs();` |
| `string` | 字符串类 | `string qrText;` |
| `ssize_t` | 有符号字节数（POSIX 读写返回） | `ssize_t n = ::write(fd_, ...);` |

### 2.2 变量定义与初始化

- **格式**：`类型 名字 = 初值;` 或 `类型 名字{初值};`（C++11 列表初始化）
- **用法**：局部变量一定要初始化，否则是随机值。`{}` 初始化更严格（会阻止窄化转换）。
- **例子**：
  ```cpp
  int hitCount = 0;
  double triArea = 0, sqArea = 0, cirArea = 0;   // 一行定义多个
  Mat gray;
  ```

### 2.3 `const` 常量

- **格式**：`const 类型 名字 = 值;`
- **用法**：只读，编译期或运行期不可改。函数参数用 `const T&` 表示"只读引用"。
- **例子**：
  ```cpp
  static const double SHARP_MIN = 18.0;        // 函数内常量
  static double sharpnessOf(const Mat& gray);  // 参数只读，不拷贝
  ```

### 2.4 `auto` 自动类型推导

- **格式**：`auto 名字 = 表达式;`
- **用法**：让编译器根据右边推类型，适合长类型名和迭代器。
- **例子**：
  ```cpp
  for (const auto& p : pts) { ... }   // p 的类型自动是 Point
  auto d = [](const Point2f& a, const Point2f& b) { return (float)norm(a-b); };
  ```

### 2.5 类型转换

- **格式**：C 风格 `(目标类型)表达式`；C++ 风格 `static_cast<目标类型>(表达式)`
- **用法**：整数除法丢小数、浮点转整数截断都要显式写。项目里大量用 C 风格，学习时也建议认识 `static_cast`。
- **例子**：
  ```cpp
  double s = (frame.cols > 640) ? 640.0 / frame.cols : 1.0;  // 640.0 保证浮点除
  (int)approx.size() == 4
  (float)margin
  (unsigned char)data[0]     // 传给 isdigit 防止负数越界
  ```

---

## 3. 运算符与表达式

### 3.1 算术与比较

`+ - * / %`，`== != < > <= >=`。注意 **整数除法**：`640 / frame.cols` 两个 int 结果是整数（丢小数），写 `640.0 /` 才得到浮点。

### 3.2 逻辑运算与短路

`&&`（与）、`||`（或）、`!`（非）。`&&`/`||` 会短路：左边决定结果就不算右边。

- **例子**：
  ```cpp
  bool detected = (sharp >= SHARP_MIN) && (qr.detect(gray, pts) && pts.size() == 4);
  // 锐度不够时后面的 qr.detect 根本不会执行——既省时间也避免无意义检测
  ```

### 3.3 位运算与标志位

- **格式**：`a | b`（按位或，常用作"组合多个标志"）、`a & b`（按位与）、`a ^ b`、`~a`、`a << n`、`a >> n`
- **用法**：很多库函数用一个 int 的每个二进制位表示不同选项，用 `|` 拼。
- **例子**：
  ```cpp
  threshold(img, bw, 0, 255, THRESH_BINARY | THRESH_OTSU);  // 二值 + Otsu 自动阈值
  open(dev, O_RDWR | O_NOCTTY | O_NDELAY);                  // 读写 + 不当控制终端 + 非阻塞
  int bits = TIOCM_DTR | TIOCM_RTS;                         // 两个控制线一起置位
  ```

### 3.4 三目运算符 `? :`

- **格式**：`条件 ? 成立时的值 : 不成立时的值`
- **用法**：替代简单的 if-else 赋值。
- **例子**：
  ```cpp
  const int aimMs = isLeft ? TURN_LEFT_LONG_MS : TURN_LEFT_SHORT_MS;
  const int backMs = isLeft ? TURN_LEFT_SHORT_MS : TURN_LEFT_LONG_MS;
  direction = (direction == "right") ? "left" : "right";   // 一句话取反
  ```

### 3.5 自增与复合赋值

`++i`、`i++`、`+=`、`-=`、`*=`、`/=`。项目中 `for (...; ++i)` 和 `nowMs() - lastFrame >= 3` 都用到。

---

## 4. 控制流

### 4.1 `if / else if / else`

```cpp
if (sharp < SHARP_MIN) {
    hint += "  TOO BLURRY - hold steady";
} else if (!detected) {
    hint += "  No QR in frame";
} else {
    polylines(frame, pts, true, Scalar(0, 255, 255), 2);
}
```

### 4.2 `for` 循环（三种常见写法）

| 写法 | 格式 | 场景 |
|---|---|---|
| 计数循环 | `for (int i = 0; i < n; ++i)` | 遍历下标：`for (size_t i = 0; i < contours.size(); ++i)` |
| 范围 for | `for (const auto& x : 容器)` | 只读遍历：`for (const auto& p : quad) cen += p;` |
| 初始化列表遍历 | `for (int v : {70, 90})` | 有限几个候选值：`for (int darkThr : { 70, 90 })` |

### 4.3 `while` 循环

- **格式**：`while (条件) { ... }`
- **用法**：条件先判断；`while (true)` 是死循环，靠内部 `break` / `return` 退出。
- **例子**：
  ```cpp
  while (true)                       // 二维码识别：没识别到就一直转
  {
      Mat frame;
      if (!cap.read(frame) || frame.empty()) continue;   // 读失败就跳过本轮
      ...
      if (!qrText.empty()) break;    // 识别成功退出
  }
  ```

### 4.4 `break` / `continue` / `return`

- `break`：跳出当前这层循环；`continue`：跳过本轮剩余语句，直接下一轮；`return`：立即返回函数（可带值）。
- **例子**：黑底筛选里 `if (a < minBase) continue;` —— 不合格的候选直接跳过，不打断整个搜索。

---

## 5. 函数

### 5.1 定义与调用

- **格式**：
  ```cpp
  返回类型 函数名(参数列表) { 函数体; return 值; }
  ```
- **用法**：参数传递方式决定"能不能改实参、要不要拷贝"：

| 写法 | 含义 | 例子 |
|---|---|---|
| `int x` | 传值（拷贝一份） | `void sleepMs(int ms)` |
| `Mat& m` | 引用（可修改实参） | `static string judge(Mat& frame, ...)` 会在图上画标注 |
| `const Mat& m` | 常引用（只读、不拷贝，最常用） | `static Mat unsharp(const Mat& gray)` |
| `const char* s` | C 风格字符串 | `static void saveImage(const Mat& img, const char* prefix, ...)` |

### 5.2 默认参数

- **格式**：`返回类型 f(类型 a, 类型 b = 默认值)`（默认值只能写在声明处）
- **例子**：OpenCV 大量使用，如 `waitKey(int delay = 0)`、`imread(const String& filename, int flags = IMREAD_COLOR)`。

### 5.3 函数重载

- **格式**：同名函数，参数列表不同（类型或个数）。
- **例子**：`rectangle(img, Rect, color, thickness)` 和 `rectangle(img, Point, Point, color, thickness)`；`getPerspectiveTransform(Point2f[], Point2f[])` 和 `getPerspectiveTransform(InputArray, InputArray)`。

### 5.4 函数必须先声明后使用

C++ 是单遍编译：调用处之前必须见过定义（或在头文件里声明）。项目里所有工具函数按依赖顺序从前往后排，就是为了这个。

---

## 6. 数组、vector 与 string

### 6.1 原生数组

- **格式**：`类型 名字[长度];`
- **例子**：`char ts[64];`、`Mat ch[3];`（存 3 个通道）、`Point2f p4[4];`

### 6.2 `vector` 动态数组

| 操作 | 格式 | 例子 |
|---|---|---|
| 定义 | `vector<T> v;` | `vector<Point> contours;` |
| 初始化列表 | `vector<T> v = {a, b, c};` | `const vector<Point2f> dstQuad = { {x,y}, ... };` |
| 二维 | `vector<vector<T>> v;` | `vector<vector<Point>> contours;`、`vector<vector<int>> cost;` |
| 取元素 | `v[i]` 或 `v.at(i)` | `contours[i]` |
| 大小/判空 | `v.size()` / `v.empty()` | `if (baseContour.empty()) return out;` |
| 追加/弹出 | `v.push_back(x)` / `v.pop_back()` | 二维码锐度候选帧缓存 |
| 原位构造 | `v.emplace_back(a, b)` | `src.emplace_back((float)p.x, (float)p.y);` |
| 预留容量 | `v.reserve(n)` | `outP.reserve(warped.size());` |
| 整体赋值 | `v.assign(begin, end)` | `quad.assign(approx.begin(), approx.end());` |
| 排序 | `sort(v.begin(), v.end(), 比较);` | 候选帧按锐度降序 |

### 6.3 `string` 常用成员

| 操作 | 格式 | 例子 |
|---|---|---|
| 查找 | `s.find(子串)`，找不到返回 `string::npos` | `if (data.find(':') == string::npos) return false;` |
| 截取 | `s.substr(起始, 长度)` | `s.substr(0, pos)`、`s.substr(pos + 1)` |
| 判空/清空 | `s.empty()` / `s.clear()` | `if (qrText.empty()) break;` |
| C 字符串 | `s.c_str()` | 传给 `atoi`、`snprintf` |
| 追加 | `s += "文字"` | `hint += "  No QR in frame";` |
| 删除字符 | `s.erase(remove_if(...), s.end())` | 解析二维码时去掉空格 |

---

## 7. 结构体、类与运算符重载

### 7.1 `struct` 与默认成员初始化

- **格式**：
  ```cpp
  struct 名字 {
      类型 成员 = 默认值;
      返回类型 成员函数(参数) { ... }
  };
  ```
- **用法**：把一组相关数据打包。C++ 的 `struct` 和 `class` 只差默认访问权限（struct 默认 public）。
- **例子**：
  ```cpp
  struct DetectedShapes
  {
      bool valid = false;
      vector<Point> tri, sq, cir;
      Rect box;
  };
  DetectedShapes out;      // 直接用，valid 已是 false
  out.valid = true;        // 用 . 访问成员
  ```

### 7.2 `class` 与访问控制、构造函数

- **格式**：
  ```cpp
  class 名字 {
  public:
      名字(参数) { 初始化 }
      返回类型 方法(参数) { ... }
  private:
      类型 成员_ = 默认值;
  };
  ```
- **例子**（项目的串口类 `SimpleSerial`）：
  ```cpp
  class SimpleSerial
  {
  public:
      bool openPort(const string& dev, int baud, bool raiseDtr);
      bool writeLine(const string& line);
      void closePort();
      bool isOpen() const { return fd_ >= 0; }   // const 成员函数：不修改对象
  private:
      int fd_ = -1;                              // 私有成员，外部只能通过方法访问
  };
  ```
  `fd_` 后面的下划线是命名约定，表示"私有成员"。

### 7.3 运算符重载：`operator>`

- **格式**：`bool operator>(const 类型& 其它) const { ... }`
- **用法**：让自定义类型支持 `>`、`<` 等，标准库算法/容器才能用。
- **例子**：A* 的优先队列要按 `f` 值从小到大出队：
  ```cpp
  struct AStarNode
  {
      int r, c, g, f;
      bool operator>(const AStarNode& o) const { return f > o.f; }
  };
  priority_queue<AStarNode, vector<AStarNode>, greater<AStarNode>> open;  // 小的先出
  ```

---

## 8. 指针、引用与内存模型

### 8.1 引用 `&`

- **格式**：`类型& 名字 = 已有变量;`（必须初始化，之后就是那个变量的别名）
- **用法**：函数参数用引用避免拷贝，同时能修改实参。
- **例子**：
  ```cpp
  static bool qrStage(VideoCapture& cap, string& qrText);  // 内部会重新打开 cap、写回 qrText
  static void turnFor(SimpleSerial& s, const char* dir, int ms, const char* desc);
  ```

### 8.2 指针 `*`

- **格式**：`类型* 名字;` 取地址 `&变量`，解引用 `*指针`
- **例子**：`DIR* dir = opendir("/dev");`（C 库返回指针，可能为 `nullptr`，用完要 `closedir`）。

### 8.3 `new/delete` 在项目中没用到

vector、Mat、string 都自己管理内存（RAII）。**你不需要手写 new/delete**，这是好事——比赛代码尽量别手动管内存。

### 8.4 OpenCV 的 `Mat` 是"浅拷贝 + 引用计数"

- `Mat b = a;` 只是让 b 指向同一块像素！
- 要独立副本必须 `a.clone()` 或 `a.copyTo(b)`。
- **例子**：`Mat crop = gray(Rect(x0,y0,w,h)).clone();` —— ROI 切片本身是视图，加 `.clone()` 才真正拷贝出来。

---

## 9. Lambda 表达式

- **格式**：
  ```cpp
  [捕获列表](参数列表) -> 返回类型 { 函数体 }
  ```
  - 捕获：`[&]` 按引用捕获外部变量，`[=]` 按值捕获，`[&cen]` 只捕获 cen 的引用，`[inv]` 按值捕获 inv。
- **用法**：写一次性的小函数，直接传给 `sort`、`find_if`，或当局部工具函数。
- **例子 1**：排序比较器
  ```cpp
  sort(cands.begin(), cands.end(),
       [](const Cand& a, const Cand& b) { return a.sharp > b.sharp; });
  ```
- **例子 2**：局部工具函数，捕获外面的矩阵
  ```cpp
  auto mapBack = [&Hinv](const vector<Point>& c) {
      vector<Point2f> src, warped;
      for (const auto& p : c) src.emplace_back((float)p.x, (float)p.y);
      perspectiveTransform(src, warped, Hinv);
      vector<Point> outP;
      for (const auto& p : warped) outP.emplace_back(cvRound(p.x), cvRound(p.y));
      return outP;
  };
  ```
- **例子 3**：按值捕获常量，避免悬空
  ```cpp
  const double inv = 1.0 / s;
  auto upscale = [inv](vector<Point>& c) { for (auto& p : c) { p.x = cvRound(p.x * inv); ... } };
  ```

---

## 10. 标准库容器与算法

### 10.1 `priority_queue` 优先队列

- **格式**：`priority_queue<T, 底层容器, 比较器> 名字;`
- **用法**：默认是最大堆；`greater<T>` 变最小堆。
- **例子**：`priority_queue<AStarNode, vector<AStarNode>, greater<AStarNode>> open;` —— A* 每次弹出 f 最小的节点。

### 10.2 `pair`

- **格式**：`pair<类型1, 类型2>`，成员 `.first` / `.second`
- **例子**：`vector<vector<pair<int,int>>> parent;` 记录每个格子的前驱坐标；`parent[p.first][p.second]`。

### 10.3 `sort` / `reverse` / `max` / `min`

- `sort(begin, end, 比较器)`：`sort(quad.begin(), quad.end(), lambda);`
- `reverse(begin, end)`：A* 回溯出的路径是倒序，`reverse(res.path.begin(), res.path.end());`
- `std::max({a,b,c})`：初始化列表版，一次比多个：`std::max({ bt.width, bt.height, bs.width });`

### 10.4 数值边界：`INT32_MAX`

`#include <cstdint>`，表示 int 的最大值。A* 用它当"无穷大"：`vector<vector<int>> gbest(R, vector<int>(C, INT32_MAX));`

---

## 11. 输入输出与格式化

### 11.1 `cout` / `cerr` / `endl`

- **格式**：`cout << 内容 << 内容;`
- **用法**：`cout` 标准输出（日志），`cerr` 标准错误（错误信息），`endl` 换行并刷新缓冲。
- **例子**：
  ```cpp
  cout << "[串口] " << desc << " -> \"" << cmd << "\"" << endl;
  cerr << "[错误] 索引 0~9 都打不开摄像头" << endl;
  ```

### 11.2 `snprintf` C 风格格式化

- **格式**：`snprintf(char buf[], size_t n, const char* fmt, ...)`
- **用法**：把格式化字符串写进固定缓冲区，`%d` 整数、`%s` 字符串、`%.1f` 一位小数、`%02d` 补零。
- **例子**：
  ```cpp
  char ts[64];
  strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &lt);      // 时间戳
  snprintf(full, sizeof(full), "result/%s_%s.png", prefix, ts);  // 拼路径
  snprintf(info, sizeof(info), "DETECTING  hit:%d  %.1fs", hitCount, elapsed);
  ```

### 11.3 `cv::format`

- **格式**：`format(const char* fmt, ...)` 返回 `String`
- **例子**：`string hint = format("sharpness=%.0f", sharp);`

### 11.4 `stringstream` 字符串流

- **格式**：`stringstream ss(字符串);` 然后 `ss >> 变量`（按空白分词）或 `getline(ss, 变量, 分隔符)`
- **用法**：解析结构化文本，比 `scanf` 安全。
- **例子**（解析二维码文本 `"8,8,0,1,7,7:1-4 0-1"`）：
  ```cpp
  stringstream head(s.substr(0, pos));
  string tok; vector<int> hv;
  while (getline(head, tok, ',')) hv.push_back(atoi(tok.c_str()));  // 逗号分割，转 int
  ```

### 11.5 `atoi` / `atof` / `isdigit` / `isspace`

- `atoi("123")` → 123；`atof("1.5")` → 1.5；`isdigit(c)` 判断是否数字字符；`isspace(c)` 判断空白。
- **例子**：`if (!data.empty() && data.find(':') != string::npos && isdigit((unsigned char)data[0]))` —— 判断这串是不是有效地图信息。

---

## 12. 时间与线程

### 12.1 `chrono` 计时

```cpp
#include <chrono>
int64_t nowMs()
{
    return (int64_t)chrono::duration_cast<chrono::milliseconds>(
               chrono::steady_clock::now().time_since_epoch()).count();
}
```
- `steady_clock::now()`：单调时钟，不受系统时间调整影响，适合测间隔。
- `duration_cast<milliseconds>`：把纳秒时长转成毫秒。
- `.count()`：取出数字。

### 12.2 `this_thread::sleep_for` 睡眠

- **格式**：`this_thread::sleep_for(chrono::milliseconds(n));`
- **例子**：`static void sleepMs(int ms) { this_thread::sleep_for(chrono::milliseconds(ms)); }`
  转弯控制的核心就是"发方向 → sleepMs(时长) → 发 P 停"。

---

## 13. POSIX 串口编程

> 这一节是 `SimpleSerial` 类的全部语法。Linux 下串口就是**设备文件**（`/dev/ttyUSB0`），用文件 IO + 终端属性配置来操作。

### 13.1 `open` 打开设备

- **格式**：`int open(const char* path, int flags);` 返回文件描述符（≥0 成功，-1 失败）
- **标志位**（用 `|` 组合）：
  | 标志 | 含义 |
  |---|---|
  | `O_RDWR` | 读写方式打开 |
  | `O_NOCTTY` | 不把该设备当成本程序的控制终端 |
  | `O_NDELAY` / `O_NONBLOCK` | 非阻塞模式 |
- **例子**：`fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);`

### 13.2 `struct termios` 终端属性

- **结构体成员**：
  | 成员 | 作用 |
  |---|---|
  | `c_iflag` | 输入模式（如 `IGNPAR` 忽略校验错） |
  | `c_oflag` | 输出模式（`~OPOST` 关掉换行转换） |
  | `c_cflag` | 控制模式（波特率、数据位、停止位、流控） |
  | `c_lflag` | 本地模式（`~ICANON` 非规范、`~ECHO` 不回显） |
  | `c_cc[NCCS]` | 特殊字符（`VMIN`、`VTIME`） |
- **常用常量**：`CS8`（8 数据位）、`CLOCAL`（忽略调制解调器线）、`CREAD`（允许接收）

### 13.3 `cfsetispeed` / `cfsetospeed` 设置波特率

- **格式**：`cfsetispeed(struct termios* t, speed_t speed);` 输入；`cfsetospeed(...)` 输出
- **波特率常量**：`B9600`、`B115200` 等（注意不是数字 9600）。
- **例子**：
  ```cpp
  cfsetispeed(&tio, B9600);
  cfsetospeed(&tio, B9600);
  ```
  项目里用 `toSpeed(int baud)` 把 9600 映射成 `B9600`。

### 13.4 `tcsetattr` 应用配置

- **格式**：`tcsetattr(int fd, int optional_actions, const struct termios* t);`
- **`optional_actions`**：`TCSANOW`（立即生效，常用）、`TCSADRAIN`（等输出发完）、`TCSAFLUSH`（清空缓冲后生效）
- **例子**：`tcsetattr(fd_, TCSANOW, &tio);`

### 13.5 `cfmakeraw` 原始模式

- **格式**：`cfmakeraw(struct termios* t);`
- **用法**：一键关掉所有行缓冲/回显/特殊字符处理，收发原始字节。单片机通信几乎都要。
- **例子**：配置完 termios 后先 `cfmakeraw(&tio);`

### 13.6 `tcflush` / `tcdrain`

| 函数 | 格式 | 作用 |
|---|---|---|
| `tcflush` | `tcflush(fd, TCIOFLUSH)` | 丢弃收发缓冲里未处理的数据 |
| `tcdrain` | `tcdrain(fd)` | 阻塞等待，直到所有已写入数据真正发出去 |

- **例子**：打开后 `tcflush(fd_, TCIOFLUSH);` 清掉残留；每次 `write` 后 `tcdrain(fd_);` 保证命令完整发出再 sleep。

### 13.7 `ioctl` + `TIOCMBIS` 控制 DTR/RTS

- **格式**：`ioctl(int fd, unsigned long request, int* arg);`
- **请求码**：`TIOCMBIS`（置位）、`TIOCMBIC`（清位）、`TIOCMGET`（读）
- **控制线**：`TIOCM_DTR`、`TIOCM_RTS`
- **例子**：某些 CH340 板子需要拉高 DTR/RTS 才会动作：
  ```cpp
  int bits = TIOCM_DTR | TIOCM_RTS;
  ::ioctl(fd_, TIOCMBIS, &bits);   // 把 DTR 和 RTS 都置高
  ```

### 13.8 `write` / `close` / `errno`

- `ssize_t write(int fd, const void* buf, size_t count);` 返回实际写入字节数
- `int close(int fd);`
- `errno` 是全局错误码，`strerror(errno)` 转成可读文字
- **例子**：发一行命令（自动补 `\r\n`）
  ```cpp
  string s = line + "\r\n";
  ssize_t n = ::write(fd_, s.data(), s.size());
  tcdrain(fd_);
  return n == (ssize_t)s.size();     // 字节数一致才算成功
  ```
  注意 `::write` 前面的 `::` 表示调用**全局**函数（防止和成员函数重名冲突）。

### 13.9 目录遍历 `opendir` / `readdir` / `closedir`

- **格式**：
  ```cpp
  DIR* dir = opendir(路径);
  struct dirent* ent;
  while ((ent = readdir(dir)) != nullptr) { ent->d_name /* 文件名 */ }
  closedir(dir);
  ```
- **例子**：`openCamera()` 里打印 `/dev/video*` 列表，帮助排查摄像头设备。

### 13.10 `mkdir` 创建目录

- **格式**：`mkdir(const char* path, mode_t mode);` 返回 0 成功，目录已存在会失败（可忽略）
- **例子**：`ensureResultDir() { ::mkdir("result", 0777); }`

---

## 14. OpenCV 核心类型

### 14.1 `Mat` 图像矩阵

| 操作 | 格式 | 例子 |
|---|---|---|
| 定义 | `Mat m;` | `Mat gray;` |
| 尺寸 | `m.cols` / `m.rows` / `m.size()` | `frame.size()` |
| 判空 | `m.empty()` | `if (img.empty()) return "";` |
| 元素类型 | `m.type()` / `m.channels()` | 一般不直接看 |
| 独立拷贝 | `a.clone()` / `a.copyTo(b)` | `Mat crop = gray(roi).clone();` |
| ROI 切片 | `m(Rect(x, y, w, h))` | `gray(Rect(x0, y0, w, h))` |
| 填充 | `m.setTo(Scalar(255))` | `paperMask.setTo(Scalar(255));` |
| 全零初始化 | `Mat::zeros(rows, cols, 类型)` | `Mat mask = Mat::zeros(size, CV_8UC1);` |

**关键概念**：`Mat` 是"头 + 像素数据指针 + 引用计数"。赋值/传参默认只拷头，不拷像素。

`CV_8UC1` 表示 8 位无符号单通道（灰度图）；`CV_8UC3` 是彩色三通道。

### 14.2 `Scalar` 颜色/数值容器

- **格式**：`Scalar(b, g, r)`（**顺序是 BGR！**），灰度用 `Scalar(v)`
- **例子**：
  ```cpp
  Scalar(0, 255, 0)      // 纯绿（B=0,G=255,R=0）
  Scalar(0, 0, 255)      // 纯红
  Scalar(0, 100, 255)    // 橙
  ```

### 14.3 `Point` / `Point2f`

- **格式**：`Point(x, y)` 整数；`Point2f(x, y)` 浮点
- **运算**：支持 `+`、`-`、`*标量`，`norm(a - b)` 求距离。
- **例子**：`Point2f cen(0, 0); for (const auto& p : quad) cen += p;`（求四角中心）。

### 14.4 `Rect` / `Size` / `RotatedRect`

| 类型 | 构造 | 成员 | 例子 |
|---|---|---|---|
| `Rect` | `Rect(x, y, w, h)` | `.x .y .width .height` | `patternROI = boundingRect(contour);` |
| `Size` | `Size(w, h)` | `.width .height` | `Size(5, 5)` 高斯核 |
| `RotatedRect` | 由 `minAreaRect` 返回 | `.center .size .angle`，`.points(Point2f[])` | `minAreaRect(contour).points(p4);` |

`Rect` 支持 `|`（并集）：`out.box = boundingRect(tri) | boundingRect(sq);` —— 取三个图形的最小外接框。

### 14.5 `Moments` 图像矩

- **格式**：`Moments m = moments(轮廓);`，零阶矩 `m.m00` 是面积，质心 = `(m.m10/m.m00, m.m01/m.m00)`
- **例子**：三角形质心，用来判断"最大角顶点在质心左还是右"：
  ```cpp
  Moments m = moments(s.tri);
  Point2f centroid((float)(m.m10 / m.m00), (float)(m.m01 / m.m00));
  ```

---

## 15. 摄像头与窗口/文件

### 15.1 `VideoCapture`

| 操作 | 格式 | 例子 |
|---|---|---|
| 打开设备号 | `VideoCapture cap(index, CAP_V4L2);` | `VideoCapture cap(idx, CAP_V4L2);` |
| 打开文件 | `VideoCapture cap("a.mp4");` | 项目未用 |
| 是否成功 | `cap.isOpened()` | `if (!cap.isOpened()) return -1;` |
| 读一帧 | `cap.read(frame)` | `if (!cap.read(frame) \|\| frame.empty()) continue;` |
| 设参数 | `cap.set(CAP_PROP_XXX, 值)` | `cap.set(CAP_PROP_FRAME_WIDTH, 1280);` |
| 读参数 | `cap.get(CAP_PROP_XXX)` | `(int)cap.get(CAP_PROP_FRAME_WIDTH)` |
| 释放 | `cap.release()` | 重开摄像头前必须先 release |

**常用属性**：`CAP_PROP_FRAME_WIDTH`、`CAP_PROP_FRAME_HEIGHT`、`CAP_PROP_FPS`、`CAP_PROP_AUTOFOCUS`、`CAP_PROP_FOURCC`
**后端**：`CAP_ANY`（自动）、`CAP_V4L2`（Linux 原生）、`CAP_GSTREAMER`

### 15.2 窗口与按键

- `imshow(窗口名, 图像)`：显示
- `int waitKey(int ms)`：等待按键，返回键码；`waitKey(1)` 刷新窗口，`waitKey(0)` 一直等到按键
- `destroyWindow(窗口名)` / `destroyAllWindows()`：关闭窗口
- **例子**：`if (waitKey(20) == 27) break;` —— 27 是 ESC 的 ASCII 码。

### 15.3 读写图片文件

- `bool imwrite(文件名, 图像)`：保存，成功返回 true
- `Mat imread(文件名, IMREAD_COLOR)`：读取
- **例子**：`if (imwrite(p, img)) cout << "[存图] " << desc;`

---

## 16. 图像预处理函数

### 16.1 `cvtColor` 颜色空间转换

`void cvtColor(InputArray src, OutputArray dst, int code, int dstCn = 0);`

常用 code：`COLOR_BGR2GRAY`、`COLOR_BGR2HSV`、`COLOR_BGR2RGB`
- **例子**：`cvtColor(frame, gray, COLOR_BGR2GRAY);`

### 16.2 `split` / `merge` 通道分离合并

`void split(InputArray m, OutputArrayOfArrays mv);`
- **例子**：`Mat ch[3]; split(frame, ch);` → ch[0]=B, ch[1]=G, ch[2]=R
- **技巧**：HSV 的 V 通道 = max(B,G,R)，所以可以 `cv::max(ch[0],ch[1],v); cv::max(v,ch[2],v);` 省掉整帧 HSV 转换。

### 16.3 `threshold` 固定阈值二值化

```cpp
double threshold(InputArray src, OutputArray dst, double thresh, double maxval, int type);
```

| 参数 | 含义 |
|---|---|
| `thresh` | 阈值（与像素值同量纲，0~255） |
| `maxval` | 超过阈值时赋的值（通常 255） |
| `type` | `THRESH_BINARY`（>阈值→255）、`THRESH_BINARY_INV`（反过来）、可 `\| THRESH_OTSU` |

- **例子 1**：`threshold(v, dark, 70, 255, THRESH_BINARY_INV);` —— 亮度 <70 的变白（找黑底）
- **例子 2**：`threshold(rectV, bright, 0, 255, THRESH_BINARY | THRESH_OTSU);` —— Otsu 自动选阈值（双峰图最佳）

### 16.4 `adaptiveThreshold` 自适应阈值

```cpp
void adaptiveThreshold(InputArray src, OutputArray dst, double maxValue,
                       int adaptiveMethod, int thresholdType, int blockSize, double C);
```
- `adaptiveMethod`：`ADAPTIVE_THRESH_MEAN_C` / `ADAPTIVE_THRESH_GAUSSIAN_C`
- `blockSize`：邻域大小（奇数，如 41）；`C`：从均值减去的常数
- **例子**：光照不均时比固定阈值稳：`adaptiveThreshold(img, bw, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 41, 10);`

### 16.5 `GaussianBlur` 高斯模糊

`void GaussianBlur(InputArray src, OutputArray dst, Size ksize, double sigmaX, ...);`
- `ksize` 必须正奇数，`Size(5,5)`；sigmaX=0 时按核大小自动算
- **例子**：`GaussianBlur(v, v, Size(5, 5), 0);`（原地模糊，省一张图）

### 16.6 `morphologyEx` 形态学

```cpp
void morphologyEx(InputArray src, OutputArray dst, int op, InputArray kernel, ...);
```
- `op`：`MORPH_OPEN`（先腐蚀后膨胀去小噪点）、`MORPH_CLOSE`（补洞）、`MORPH_ERODE`、`MORPH_DILATE`
- kernel 由 `getStructuringElement(MORPH_RECT, Size(3,3))` 生成
- **例子**：`morphologyEx(dark, dark, MORPH_OPEN, openKernel);` 去掉零碎暗点

### 16.7 `resize` 缩放

`void resize(InputArray src, OutputArray dst, Size dsize, double fx=0, double fy=0, int interpolation=INTER_LINEAR);`
- 插值：`INTER_LINEAR`（默认）、`INTER_AREA`（缩小首选）、`INTER_CUBIC`（放大首选）
- **例子**：`resize(crop, c2, Size(size, size), 0, 0, INTER_CUBIC);`

### 16.8 锐化与锐度评估

```cpp
// 反锐化掩模：原图 + 权重×(原图 - 模糊版)
Mat unsharp(const Mat& gray) {
    Mat blur, out;
    GaussianBlur(gray, blur, Size(0, 0), 2.0);
    addWeighted(gray, 1.6, blur, -0.6, 0, out);   // out = 1.6*gray - 0.6*blur
    return out;
}
// 锐度 = 拉普拉斯响应的平均绝对值
double sharpnessOf(const Mat& gray) {
    Mat lap, abs16;
    Laplacian(gray, lap, CV_16S, 3);
    convertScaleAbs(lap, abs16);                  // 取绝对值并转 8 位
    return mean(abs16)[0];
}
```
- `addWeighted(src1, α, src2, β, γ, dst)`：`dst = α·src1 + β·src2 + γ`
- `Laplacian(src, dst, ddepth, ksize)`：二阶导数，边缘处响应大
- `convertScaleAbs(src, dst, α=1, β=0)`：`dst = |src·α + β|` 并饱和转 8 位
- `mean(img)` 返回 `Scalar`，`[0]` 取第一通道均值

### 16.9 `norm` / `cvRound`

- `double norm(InputArray a, InputArray b, int normType=NORM_L2)`：两图/两点距离
- `int cvRound(double)`：四舍五入取整（比 `(int)(x+0.5)` 快且正确）
- **例子**：`if (colorDiff < COLOR_DIFF_THRESH)` 里 `colorDiff = norm(cirColor - sqColor);`

---

## 17. 几何变换与透视

### 17.1 `getPerspectiveTransform` 求透视矩阵

- **格式**：`Mat H = getPerspectiveTransform(InputArray srcPts, InputArray dstPts);`（4 个点对 → 3×3 矩阵）
- **坑**：两个参数类型必须一致（都 vector 或都数组），混用会编译失败。
- **例子**：把检测到的黑底四角映射成正方形：
  ```cpp
  Mat H = getPerspectiveTransform(quad, dstQuad);   // quad/dstQuad 都是 vector<Point2f>
  ```

### 17.2 `warpPerspective` 透视变换（拉正）

- **格式**：`warpPerspective(InputArray src, OutputArray dst, InputArray M, Size dsize, int flags);`
- **例子**：`warpPerspective(v, rectV, H, Size(S, S), INTER_CUBIC);` —— 得到正视角的黑底方块图

### 17.3 `perspectiveTransform` 点集变换（含反变换）

- **格式**：`perspectiveTransform(InputArray src, OutputArray dst, InputArray m);`
- **例子**：把拉正图里的轮廓坐标映射回原画面：
  ```cpp
  Mat Hinv = H.inv();
  perspectiveTransform(srcPts, dstPts, Hinv);
  ```

### 17.4 `H.inv()` 矩阵求逆

`Mat` 的成员函数，返回逆矩阵。透视变换的逆变换 = 用逆矩阵再变换一次。

---

## 18. 轮廓处理

### 18.1 `findContours` 找轮廓

```cpp
void findContours(InputArray image, OutputArrayOfArrays contours, int mode, int method);
```

| 参数 | 可选值 | 含义 |
|---|---|---|
| `image` | 8 位单通道**二值图** | OpenCV 4 不会修改输入 |
| `contours` | `vector<vector<Point>>` | 输出：每个轮廓是一串点 |
| `mode` | `RETR_EXTERNAL`（只要最外层）/ `RETR_LIST`（全部）/ `RETR_TREE`（带层级） | 项目用 EXTERNAL |
| `method` | `CHAIN_APPROX_SIMPLE`（压缩直线段）/ `CHAIN_APPROX_NONE`（存每个点） | 项目用 SIMPLE |

- **例子**：`findContours(dark, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);`

### 18.2 `contourArea` / `arcLength`

- `double contourArea(InputArray contour, bool oriented=false)`：轮廓面积
- `double arcLength(InputArray curve, bool closed)`：周长
- **例子**：`if (a < minBase || a > 0.35 * frameArea) continue;` 用面积筛候选黑底。

### 18.3 `approxPolyDP` 多边形逼近

- **格式**：`approxPolyDP(InputArray curve, OutputArray approx, double epsilon, bool closed);`
- **用法**：把轮廓简化成少几个点，点数=3 就是三角形、4 是四边形。`epsilon` 是允许的最大偏差，常用 `0.02 * 周长`。
- **例子**：
  ```cpp
  approxPolyDP(contours[i], approx, 0.02 * arcLength(contours[i], true), true);
  if ((int)approx.size() == 3) return 0;   // 三角形
  ```

### 18.4 `boundingRect` / `minAreaRect`

- `Rect boundingRect(InputArray pts)`：正矩形外接框
- `RotatedRect minAreaRect(InputArray pts)`：最小面积旋转矩形（带角度）
- **例子**：实心度 = 面积 / 旋转矩形面积，判断黑底是否"实心方块"：
  ```cpp
  RotatedRect rr = minAreaRect(contour);
  double fill = a / (rr.size.width * rr.size.height + 1e-6);
  if (fill < 0.6) continue;
  ```

### 18.5 `drawContours` / `fillPoly`

- `drawContours(img, contours, 索引, 颜色, thickness)`：thickness=-1 表示填充
- `fillPoly(img, {轮廓}, 颜色)`：填充多边形（项目用来生成掩膜）
- **例子**：`Mat mask = Mat::zeros(bgr.size(), CV_8UC1); fillPoly(mask, vector<vector<Point>>{contour}, Scalar(255));`

### 18.6 `meanColorOf` 里的三个函数

```cpp
static Scalar meanColorOf(const Mat& bgr, const vector<Point>& contour)
{
    Mat mask = Mat::zeros(bgr.size(), CV_8UC1);
    fillPoly(mask, vector<vector<Point>>{contour}, Scalar(255));   // 轮廓内=255
    Mat eroded;
    erode(mask, eroded, getStructuringElement(MORPH_RECT, Size(5, 5)));  // 内缩，避开边缘混色
    return mean(bgr, (countNonZero(eroded) > 0) ? eroded : mask); // 只统计掩膜内像素均值
}
```
- `erode`：腐蚀（白色区域缩小），避开轮廓边缘的抗锯齿/混色
- `countNonZero`：统计非零像素个数，判断腐蚀后是否还剩东西
- `mean(src, mask)`：只对 mask 非零处求均值，返回 `Scalar(B,G,R)`

---

## 19. 绘图与文字

| 函数 | 格式要点 | 例子 |
|---|---|---|
| `rectangle` | `rectangle(img, Rect, 颜色, 线宽)` | `rectangle(frame, s.box, Scalar(0,255,0), 3);` |
| `circle` | `circle(img, 圆心, 半径, 颜色, 线宽)` | `circle(img, center, 18, Scalar(0,180,0), -1);`（-1=实心） |
| `line` | `line(img, 点1, 点2, 颜色, 线宽)` | `line(frame, centroid, maxVertex, Scalar(0,0,255), 2);` |
| `polylines` | `polylines(img, 点集, 是否闭合, 颜色, 线宽)` | `polylines(frame, pts, true, Scalar(0,255,255), 2);` |
| `putText` | `putText(img, 文字, 左下角坐标, 字体, 缩放, 颜色, 线宽)` | `putText(frame, hint, Point(20,40), FONT_HERSHEY_SIMPLEX, 0.85, 颜色, 2);` |

- 字体常用 `FONT_HERSHEY_SIMPLEX`
- `LINE_AA` 表示抗锯齿，画出来更平滑

---

## 20. 二维码专用 API

### 20.1 `QRCodeDetector`

| 方法 | 签名要点 | 用途 |
|---|---|---|
| `detect` | `bool detect(InputArray img, OutputArray points)` | 只找位置：输出 4 个角点（`Mat` 1×4×CV_32FC2） |
| `decode` | `String decode(InputArray img, InputArray points, OutputArray straight)` | 用已知角点解码 |
| `detectAndDecode` | `String detectAndDecode(InputArray img, OutputArray points=noArray(), OutputArray straight=noArray())` | 检测+解码一步到位 |
| `detectAndDecodeCurved` | 同上，OpenCV ≥ 4.5.2 | 对透视/弯曲二维码更强 |

- **例子**：
  ```cpp
  QRCodeDetector qr;
  vector<Point> pts;
  bool detected = qr.detect(gray, pts) && pts.size() == 4;   // 只要检测到角点
  string data = qr.detectAndDecode(img);                      // 一步解出内容
  ```

### 20.2 `Ptr<T>` 智能指针

- **格式**：`Ptr<类型> 名字 = 创建函数();`（OpenCV 自己的 shared_ptr）
- **用法**：自动释放，不需要 delete。
- **例子**：`Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8)); clahe->apply(gray, g);`

### 20.3 `createCLAHE` 自适应直方图均衡

- **格式**：`Ptr<CLAHE> createCLAHE(double clipLimit = 40.0, Size tileGridSize = Size(8,8));`
- **用法**：分块均衡，改善局部对比度（暗处细节）。`clipLimit` 越大约束越弱。
- **例子**：`clahe->apply(gray, g);`（离线兜底时使用）

### 20.4 `zbar`（可选依赖）

- `zbar::ImageScanner scanner; scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);`
- `zbar::Image imgZ(cols, rows, "Y800", data, size);`
- `scanner.scan(imgZ); for (auto it = imgZ.symbol_begin(); ...) it->get_data();`
- **用法**：比 OpenCV 内置解码器鲁棒，是这台机器上真正能解出码的途径。

---

## 21. 综合案例：detectShapes 逐行拆解

把上面语法串起来看一遍（简化版），理解"语法是用来表达思路的"：

```cpp
static DetectedShapes detectShapes(const Mat& frame)   // 5.1 常引用参数
{
    DetectedShapes out;                                // 7.1 结构体默认初始化

    Mat ch[3];                                         // 6.1 原生数组
    split(frame, ch);                                  // 16.2 通道分离
    Mat v;
    cv::max(ch[0], ch[1], v);                          // 16.2 逐像素取最大
    cv::max(v, ch[2], v);                              // → V 通道
    GaussianBlur(v, v, Size(5, 5), 0);                 // 16.5 去噪

    const double s = (frame.cols > 640) ? 640.0 / frame.cols : 1.0;  // 3.4 三目
    const double frameArea = (double)v.rows * v.cols;  // 2.5 类型转换

    vector<Point> baseContour;                         // 6.2 vector
    double baseArea = 0;
    for (int darkThr : {70, 90})                       // 4.2 初始化列表遍历
    {
        Mat dark;
        threshold(v, dark, darkThr, 255, THRESH_BINARY_INV);   // 3.3 位标志/16.3
        morphologyEx(dark, dark, MORPH_OPEN, openKernel);      // 16.6
        vector<vector<Point>> contours;
        findContours(dark, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);  // 18.1

        for (size_t i = 0; i < contours.size(); ++i)   // 2.1 size_t
        {
            const double a = contourArea(contours[i]); // 18.2
            if (a < 0.002 * frameArea) continue;       // 4.4 continue
            const Rect r = boundingRect(contours[i]);  // 18.4
            const double ar = (double)r.width / std::max(r.height, 1);
            if (ar < 0.35 || ar > 2.8) continue;
            if (a > baseArea) { baseArea = a; baseContour = contours[i]; }
        }
        if (!baseContour.empty()) break;               // 4.4 break
    }
    if (baseContour.empty()) return out;               // 4.4 return
    ...
    const Mat H = getPerspectiveTransform(quad, dstQuad);   // 17.1
    warpPerspective(v, rectV, H, Size(S, S), INTER_CUBIC);  // 17.2
    ...
    auto mapBack = [&Hinv](const vector<Point>& c) { ... }; // 9 Lambda
    out.tri = mapBack(triR);                            // 5.1 返回值
    out.valid = !out.tri.empty() && !out.sq.empty();    // 3.2 短路与
    return out;
}
```

**读法**：先看函数签名知道输入输出，再看每个局部变量是什么类型，再看控制流（for/if/break），最后看每步调用了哪个库函数。语法只是外壳，思路是"降噪 → 找暗块 → 筛形状 → 拉正 → 分类 → 映射回来"。

---

## 22. 常见坑与学习建议

### 常见坑

1. **BGR 不是 RGB**：`Scalar(0,255,0)` 是绿色不是红色。红色是 `Scalar(0,0,255)`。
2. **整数除法**：`640 / w` 两个 int 会丢小数，要写 `640.0 / w`。
3. **`Mat` 赋值是浅拷贝**：`Mat b = a;` 改 b 会影响 a，需要副本就用 `.clone()`。
4. **`findContours` 只能吃二值图**：输入必须是 8 位单通道。
5. **`waitKey` 必须有**：`imshow` 后不调 `waitKey`，窗口不刷新。
6. **摄像头重开前要 `release()`**：否则同一个设备可能打不开。
7. **位标志用 `|`，不要用 `+`**：`THRESH_BINARY | THRESH_OTSU`。
8. **`getPerspectiveTransform` 两个参数类型必须一致**（都 vector 或都数组）。
9. **`size()` 返回 `size_t`（无符号）**：`for (int i = size-1; i >= 0; --i)` 有符号坑，用 `size_t` 或 `int` 显式转。
10. **结构体在函数里定义**：像 `struct Cand { ... }` 只能在该函数内用。

### 学习建议（按依赖顺序）

1. 先啃通 **第 2、3、4 章**（变量/运算/控制流）——所有语言通用，半天够了。
2. 再看 **第 5、6、7 章**（函数/容器/结构体）——能读懂代码结构。
3. **第 14、16、18 章**（Mat/预处理/轮廓）是 OpenCV 的骨架，边看边改参数跑一遍最有效。
4. **第 13 章**串口偏系统编程，记住"打开→配置→读写→关闭"四步即可，细节可以查手册。
5. 最后 **第 17、20 章**（透视/二维码）是对前面积累的综合运用。

一个高效练习：把 `detectShapes` 里的某个阈值（比如 `ar < 0.35`）改成别的值，观察识别结果变化——比看十遍文档都管用。
