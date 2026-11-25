/*********************************************************************
 * 最短路径识别（4.2）
 * ------------------------------------------------------------------
 * 功能流程：
 *   1. 打开摄像头，识别约 2 米外白纸上的二维码（智能策略：
 *      锐度挑帧 + 角点透视矫正 + 渐进式多尺寸解码 + zbar 兜底）
 *   2. 解析二维码文本：
 *        格式: 行数,列数,起点行,起点列,终点行,终点列:值-数量 值-数量 ...
 *        冒号前 -> 地图尺寸与起终点；冒号后按空格分割，每个 '值-数量'
 *        展开后按【行优先】顺序填充地图。值 0 = 障碍物（不可通行），1~8 = 通行代价（8 是高代价可通行区）
 *   3. A* 算法搜索总代价最小的路径（四方向移动，进入格子的代价 = 该格代价）
 *   4. 用 OpenCV 画出栅格地图、障碍物、代价、起终点与最短路线
 *
 * 编译（示例）:
 *   g++ path_planner.cpp -o path_planner `pkg-config --cflags --libs opencv4`
 *   或 CMake 链接 OpenCV 即可
 *
 * 运行:
 *   ./path_planner            # 默认打开摄像头 0
 *   ./path_planner 1          # 打开摄像头 1
 *   ./path_planner paper.jpg  # 直接识别图片文件中的二维码
 *   ./path_planner "8,8,0,1,7,7:1-4 0-1 ..."   # 直接解析二维码文本（调试用）
 *********************************************************************/

#include <opencv2/opencv.hpp>
#ifdef HAVE_ZBAR
#include <zbar.h>
#endif
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

static const int OBSTACLE = 0;      // 障碍物代价值（不可通行）；其余 1~8 为通行代价

static bool isBlocked(int v) { return v == 0; }   // 只有 0 是障碍物，8 可以走（代价 8）

/* ===================================================================
 * 一、二维码识别（智能策略：快路径优先 + 透视矫正 + 渐进式解码）
 * ------------------------------------------------------------------
 * 设计思路（2 米外 10 秒内稳定识别）：
 *   · 每帧先走零成本快路径（原图直解），清晰近距时 1 帧命中
 *   · 角点检测成功后【优先透视矫正】：把畸变二维码拉正，一次顶十次盲试
 *   · 矫正图上按 多尺寸 × 多变体 渐进解码，按性价比排序，命中即停
 *   · zbar（若安装）在每一步并行兜底，比 OpenCV 解码器鲁棒得多
 * =================================================================== */
static string tryDecode(const Mat& img, QRCodeDetector& qr)
{
    if (img.empty()) return "";
    // detectAndDecodeCurved 对透视畸变/倾斜二维码更强（OpenCV >= 4.5.2）
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR > 5) \
    || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR == 5 && CV_VERSION_PATCH >= 2)
    {
        Mat points, straight;
        string data = qr.detectAndDecodeCurved(img, points, straight);
        if (!data.empty() && data.find(':') != string::npos &&
            isdigit((unsigned char)data[0]))
            return data;
    }
#endif
    string data = qr.detectAndDecode(img);
    // 只有包含冒号且以数字开头的串才认为是有效地图信息
    if (!data.empty() && data.find(':') != string::npos &&
        isdigit((unsigned char)data[0]))
        return data;
    return "";
}

// 锐化（USM 反锐化掩模）：增强黑白模块边缘，远距离拍摄时很有效
static Mat unsharp(const Mat& gray)
{
    Mat blur, out;
    GaussianBlur(gray, blur, Size(0, 0), 2.0);
    addWeighted(gray, 1.6, blur, -0.6, 0, out);
    return out;
}

// 锐度评估（平均绝对拉普拉斯响应）：太低说明运动模糊/失焦，解码大概率白费
static double sharpnessOf(const Mat& gray)
{
    Mat lap, abs16;
    Laplacian(gray, lap, CV_16S, 3);
    convertScaleAbs(lap, abs16);
    return mean(abs16)[0];
}

#ifdef HAVE_ZBAR
// zbar 解码（比 OpenCV 鲁棒得多）：输入需为 8 位灰度连续内存
static string zbarDecode(const Mat& gray)
{
    if (gray.empty()) return "";
    Mat wrapped;
    if (gray.isContinuous()) wrapped = gray;
    else gray.copyTo(wrapped);
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    zbar::Image imgZ(wrapped.cols, wrapped.rows, "Y800",
                     (uchar*)wrapped.data, wrapped.cols * wrapped.rows);
    scanner.scan(imgZ);
    for (zbar::Image::SymbolIterator it = imgZ.symbol_begin();
         it != imgZ.symbol_end(); ++it)
    {
        string d = it->get_data();
        if (!d.empty() && d.find(':') != string::npos &&
            isdigit((unsigned char)d[0]))
            return d;
    }
    return "";
}
#endif

// 把 4 个角点按极角排成环序（无论检测器给什么顺序都能用于透视变换）
static void orderQuad(const vector<Point>& pts, vector<Point2f>& quad)
{
    Point2f cen(0, 0);
    for (const auto& p : pts) cen += Point2f(p);
    cen.x /= 4; cen.y /= 4;
    quad.clear();
    for (const auto& p : pts) quad.push_back(Point2f(p));
    sort(quad.begin(), quad.end(), [&](const Point2f& a, const Point2f& b) {
        return atan2(a.y - cen.y, a.x - cen.x) < atan2(b.y - cen.y, b.x - cen.x);
    });
}

// 透视矫正图上的渐进式解码：多尺寸 × 多变体，按性价比排序，命中即停
static string decodeRectified(const Mat& flat, QRCodeDetector& qr)
{
    string data;
    // 尺寸序列覆盖不同模块密度（QR 版本 4~12 大致对应 300~1100px）
    for (int size : {640, 460, 880, 320, 1100})
    {
        Mat img;
        resize(flat, img, Size(size, size), 0, 0, INTER_CUBIC);
        if ((data = tryDecode(img, qr)).size()) return data;
        if ((data = tryDecode(unsharp(img), qr)).size()) return data;
        Mat bw;
        threshold(img, bw, 0, 255, THRESH_BINARY | THRESH_OTSU);
        if ((data = tryDecode(bw, qr)).size()) return data;
        adaptiveThreshold(img, bw, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
                          THRESH_BINARY, 41, 10);
        if ((data = tryDecode(bw, qr)).size()) return data;
#ifdef HAVE_ZBAR
        if ((data = zbarDecode(img)).size()) return data;
        if ((data = zbarDecode(bw)).size()) return data;
#endif
    }
    return "";
}

// 用检测到的 4 角点把二维码透视拉正后解码——检测成功时性价比最高的一步
static string decodeViaQuad(const Mat& gray, const vector<Point>& pts,
                            QRCodeDetector& qr)
{
    vector<Point2f> quad;
    orderQuad(pts, quad);
    auto d = [](const Point2f& a, const Point2f& b) { return (float)norm(a - b); };
    float side = max({ d(quad[0],quad[1]), d(quad[1],quad[2]),
                       d(quad[2],quad[3]), d(quad[3],quad[0]) });
    int S = min(max((int)(side * 6) + 64, 300), 1600);   // 放大 + 静区
    vector<Point2f> dst = { {24,24}, {S-24,24}, {S-24,S-24}, {24,S-24} };
    Mat H = getPerspectiveTransform(quad, dst);
    Mat flat;
    warpPerspective(gray, flat, H, Size(S, S), INTER_CUBIC);
    return decodeRectified(flat, qr);
}

// 智能解码一帧灰度图：快路径 → 透视矫正 → ROI 放大 → zbar → 全图兜底
static string decodeFrameSmart(const Mat& gray, QRCodeDetector& qr)
{
    string data;

    // 0) 零成本快路径：清晰、近距时直接命中
    if ((data = tryDecode(gray, qr)).size()) return data;
    if ((data = tryDecode(unsharp(gray), qr)).size()) return data;

    vector<Point> pts;
    bool detected = (qr.detect(gray, pts) && pts.size() == 4);
    if (detected)
    {
        // ① 透视矫正解码（角点已知时，这一步胜过任何盲目放大）
        if ((data = decodeViaQuad(gray, pts, qr)).size()) return data;

        // ② ROI 裁剪放大（矫正失败的备胎：光照极端不均时仍有价值）
        float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
        for (const auto& p : pts) {
            minx = min(minx, (float)p.x); miny = min(miny, (float)p.y);
            maxx = max(maxx, (float)p.x); maxy = max(maxy, (float)p.y);
        }
        int pad = (int)std::max(maxx - minx, maxy - miny);
        int x0 = max(0, (int)minx - pad), y0 = max(0, (int)miny - pad);
        int x1 = min(gray.cols, (int)maxx + pad), y1 = min(gray.rows, (int)maxy + pad);
        Mat crop = gray(Rect(x0, y0, x1 - x0, y1 - y0)).clone();
        for (int size : {500, 800})
        {
            Mat c2;
            resize(crop, c2, Size(size, size), 0, 0, INTER_CUBIC);
            if ((data = tryDecode(c2, qr)).size()) return data;
            if ((data = tryDecode(unsharp(c2), qr)).size()) return data;
            Mat bw;
            threshold(c2, bw, 0, 255, THRESH_BINARY | THRESH_OTSU);
            if ((data = tryDecode(bw, qr)).size()) return data;
        }
    }

    // ③ zbar 全图兜底（原图 + 2x 放大）
#ifdef HAVE_ZBAR
    if ((data = zbarDecode(gray)).size()) return data;
    {
        Mat up;
        resize(gray, up, Size(), 2, 2, INTER_CUBIC);
        if ((data = zbarDecode(up)).size()) return data;
    }
#endif

    // ④ 全图多尺度兜底（轻度，只试性价比最高的两档）
    for (double scale : {2.0, 3.0})
    {
        Mat up;
        resize(gray, up, Size(), scale, scale, INTER_CUBIC);
        GaussianBlur(up, up, Size(3, 3), 0);
        if ((data = tryDecode(up, qr)).size()) return data;
        Mat bw;
        threshold(up, bw, 0, 255, THRESH_BINARY | THRESH_OTSU);
        if ((data = tryDecode(bw, qr)).size()) return data;
    }
    return "";
}

// 兼容旧接口（图片文件 / 调试模式）：智能解码 + 离线重火力兜底
static string decodeQRCodeMultiScale(Mat src)
{
    QRCodeDetector qr;
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);

    string data = decodeFrameSmart(gray, qr);
    if (data.size()) return data;

    // 离线场景时间不限：补上 CLAHE 与完整多尺度
    Mat g;
    Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8));
    clahe->apply(gray, g);
    if ((data = tryDecode(g, qr)).size()) return data;
    for (double scale : {1.2, 1.5, 2.5})
        for (const Mat& prep : { gray, unsharp(gray) })
        {
            Mat up;
            resize(prep, up, Size(), scale, scale, INTER_CUBIC);
            GaussianBlur(up, up, Size(3, 3), 0);
            if ((data = tryDecode(up, qr)).size()) return data;
            Mat bw;
            threshold(up, bw, 0, 255, THRESH_BINARY | THRESH_OTSU);
            if ((data = tryDecode(bw, qr)).size()) return data;
            adaptiveThreshold(up, bw, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
                              THRESH_BINARY, 51, 10);
            if ((data = tryDecode(bw, qr)).size()) return data;
        }
    return "";
}

/* ===================================================================
 * 二、二维码文本解析 -> 栅格地图
 * =================================================================== */
struct GridMap
{
    int rows = 0, cols = 0;
    Point start, goal;
    vector<vector<int>> cost;                 // 代价，0 表示障碍（不可通行）
};

static bool parseQRString(const string& s, GridMap& m)
{
    size_t pos = s.find(':');
    if (pos == string::npos) return false;

    // ---- 冒号前：行数,列数,起点行,起点列,终点行,终点列 ----
    stringstream head(s.substr(0, pos));
    string tok; vector<int> hv;
    while (getline(head, tok, ',')) {
        tok.erase(remove_if(tok.begin(), tok.end(), ::isspace), tok.end());
        if (!tok.empty()) hv.push_back(atoi(tok.c_str()));
    }
    if (hv.size() < 6) { cerr << "[解析] 冒号前字段不足 6 个\n"; return false; }
    m.rows = hv[0]; m.cols = hv[1];
    m.start = Point(hv[3], hv[2]);            // (列, 行) -> OpenCV (x, y)
    m.goal  = Point(hv[5], hv[4]);
    if (m.rows <= 0 || m.cols <= 0) return false;

    // ---- 冒号后：按空格分割，'值-数量' 展开后按行优先填充 ----
    vector<int> flat;
    stringstream body(s.substr(pos + 1));
    while (body >> tok)
    {
        size_t dash = tok.find('-');
        if (dash == string::npos) { cerr << "[解析] 代价格式错误: " << tok << "\n"; return false; }
        int v = atoi(tok.substr(0, dash).c_str());
        int n = atoi(tok.substr(dash + 1).c_str());
        for (int i = 0; i < n; ++i) flat.push_back(v);
    }
    if ((int)flat.size() < m.rows * m.cols) {
        cerr << "[解析] 展开后格子数 " << flat.size() << " < " << m.rows * m.cols << "\n";
        return false;
    }

    m.cost.assign(m.rows, vector<int>(m.cols, 0));
    int k = 0;
    for (int r = 0; r < m.rows; ++r)
        for (int c = 0; c < m.cols; ++c)
            m.cost[r][c] = flat[k++];         // 行优先
    return true;
}

/* ===================================================================
 * 三、A* 最短路径（四方向，进入格子花费该格代价，障碍不可走）
 * =================================================================== */
struct AStarNode          // 注意: 不能叫 Node，会与 OpenCV 内部的 cv::Node 冲突
{
    int r, c;
    int g;                       // 起点到此的实际代价
    int f;
    bool operator>(const AStarNode& o) const { return f > o.f; }
};

struct AStarResult
{
    bool found = false;
    int  totalCost = 0;
    vector<Point> path;          // (x=列, y=行)
};

static AStarResult astar(const GridMap& m)
{
    AStarResult res;
    const int R = m.rows, C = m.cols;
    auto inside = [&](int r, int c) { return r >= 0 && r < R && c >= 0 && c < C; };
    auto walk   = [&](int r, int c) { return inside(r, c) && !isBlocked(m.cost[r][c]); };

    if (!walk(m.start.y, m.start.x) || !walk(m.goal.y, m.goal.x)) {
        cerr << "[A*] 起点或终点是障碍物！\n";
        return res;
    }

    // 可采纳启发式：曼哈顿距离 × 全图最小通行代价
    int minCost = INT32_MAX;
    for (int r = 0; r < R; ++r)
        for (int c = 0; c < C; ++c)
            if (!isBlocked(m.cost[r][c])) minCost = min(minCost, m.cost[r][c]);
    if (minCost == INT32_MAX) minCost = 0;
    auto h = [&](int r, int c) { return (abs(r - m.goal.y) + abs(c - m.goal.x)) * minCost; };

    priority_queue<AStarNode, vector<AStarNode>, greater<AStarNode>> open;
    vector<vector<int>> gbest(R, vector<int>(C, INT32_MAX));
    vector<vector<pair<int,int>>> parent(R, vector<pair<int,int>>(C, {-1,-1}));
    vector<vector<bool>> closed(R, vector<bool>(C, false));

    int sr = m.start.y, sc = m.start.x, gr = m.goal.y, gc = m.goal.x;
    gbest[sr][sc] = 0;
    open.push({sr, sc, 0, h(sr, sc)});

    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = { 0, 0,-1, 1};

    while (!open.empty())
    {
        AStarNode cur = open.top(); open.pop();
        if (closed[cur.r][cur.c]) continue;
        closed[cur.r][cur.c] = true;

        if (cur.r == gr && cur.c == gc)
        {
            res.found = true;
            res.totalCost = gbest[gr][gc];
            // 回溯路径（不含起点，包含终点；代价 = 进入各格代价之和）
            pair<int,int> p = {gr, gc};
            while (!(p.first == sr && p.second == sc)) {
                res.path.push_back(Point(p.second, p.first));
                p = parent[p.first][p.second];
            }
            res.path.push_back(m.start);
            reverse(res.path.begin(), res.path.end());
            return res;
        }

        for (int d = 0; d < 4; ++d)
        {
            int nr = cur.r + dr[d], nc = cur.c + dc[d];
            if (!walk(nr, nc) || closed[nr][nc]) continue;
            int ng = gbest[cur.r][cur.c] + m.cost[nr][nc];   // 进入新格的代价
            if (ng < gbest[nr][nc]) {
                gbest[nr][nc] = ng;
                parent[nr][nc] = {cur.r, cur.c};
                open.push({nr, nc, ng, ng + h(nr, nc)});
            }
        }
    }
    return res;
}

/* ===================================================================
 * 四、OpenCV 可视化
 * =================================================================== */
static Mat drawMap(const GridMap& m, const AStarResult& res)
{
    const int CELL = 80, MARGIN = 40;
    Mat img(m.rows * CELL + 2 * MARGIN, m.cols * CELL + 2 * MARGIN, CV_8UC3,
            Scalar(245, 245, 245));

    for (int r = 0; r < m.rows; ++r)
        for (int c = 0; c < m.cols; ++c)
        {
            int x = MARGIN + c * CELL, y = MARGIN + r * CELL;
            Scalar fill = isBlocked(m.cost[r][c]) ? Scalar(40, 40, 40)   // 障碍(0)：深灰
                                                  : Scalar(255, 255, 255);
            rectangle(img, Rect(x, y, CELL, CELL), fill, -1);
            rectangle(img, Rect(x, y, CELL, CELL), Scalar(120, 120, 120), 1);
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", m.cost[r][c]);
            putText(img, buf, Point(x + CELL / 2 - 8, y + CELL / 2 + 8),
                    FONT_HERSHEY_SIMPLEX, 0.55,
                    isBlocked(m.cost[r][c]) ? Scalar(255, 255, 255)
                                            : Scalar(90, 90, 90), 1, LINE_AA);
        }

    // 最短路线（红色折线）
    if (res.found)
        for (size_t i = 1; i < res.path.size(); ++i)
            line(img,
                 Point(MARGIN + res.path[i-1].x * CELL + CELL/2,
                       MARGIN + res.path[i-1].y * CELL + CELL/2),
                 Point(MARGIN + res.path[i].x   * CELL + CELL/2,
                       MARGIN + res.path[i].y   * CELL + CELL/2),
                 Scalar(0, 0, 255), 4, LINE_AA);

    // 起点（绿）/ 终点（蓝）
    auto center = [&](Point p) { return Point(MARGIN + p.x * CELL + CELL/2,
                                              MARGIN + p.y * CELL + CELL/2); };
    circle(img, center(m.start), 18, Scalar(0, 180, 0), -1, LINE_AA);
    putText(img, "S", center(m.start) + Point(-8, 7),
            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2, LINE_AA);
    circle(img, center(m.goal), 18, Scalar(255, 120, 0), -1, LINE_AA);
    putText(img, "E", center(m.goal) + Point(-8, 7),
            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2, LINE_AA);

    // 顶部信息栏
    string info = res.found
        ? format("A* path cost = %d  steps = %d", res.totalCost, (int)res.path.size() - 1)
        : "A* path NOT found (goal unreachable)";
    putText(img, info, Point(MARGIN, 28), FONT_HERSHEY_SIMPLEX, 0.7,
            res.found ? Scalar(0, 0, 200) : Scalar(0, 0, 255), 2, LINE_AA);
    return img;
}

/* ===================================================================
 * 五、main
 * =================================================================== */
int main(int argc, char** argv)
{
    string qrText;
    bool fromCamera = true;

    if (argc >= 2)
    {
        string arg = argv[1];
        if (arg.find(':') != string::npos) {           // 直接传入二维码文本
            qrText = arg; fromCamera = false;
        } else if (arg.find_first_not_of("0123456789") == string::npos) {
            fromCamera = true;                          // 摄像头编号
        } else {                                        // 图片文件
            Mat img = imread(arg);
            if (img.empty()) { cerr << "无法读取图片: " << arg << "\n"; return 1; }
            qrText = decodeQRCodeMultiScale(img);
            fromCamera = false;
            if (qrText.empty()) { cerr << "图片中未识别到二维码\n"; return 2; }
        }
    }

    if (fromCamera)
    {
        int camIdx = (argc >= 2) ? atoi(argv[1]) : 0;
        VideoCapture cap(camIdx);
        if (!cap.isOpened()) { cerr << "无法打开摄像头 " << camIdx << "\n"; return 3; }
        cap.set(CAP_PROP_FRAME_WIDTH,  1280);          // 尽量高分辨率，远处二维码更清晰
        cap.set(CAP_PROP_FRAME_HEIGHT, 720);
        cap.set(CAP_PROP_AUTOFOCUS, 1);                // 尽力开启自动对焦（不支持的摄像头忽略）
        QRCodeDetector qr;
        cout << "摄像头已打开，请将 2 米外的白纸二维码正对镜头，保持稳定... 按 ESC 退出\n";

        // ---- 候选帧缓存：保留锐度最高的 3 个已检测帧 ----
        struct Cand { double sharp; vector<Point> quad; Mat gray; };
        vector<Cand> cands;
        long frameNo = 0, lastAttempt = -100;
        const double SHARP_MIN = 18.0;                 // 低于此值说明太模糊

        while (true)
        {
            Mat frame;
            if (!cap.read(frame) || frame.empty()) continue;
            ++frameNo;

            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            double sharp = sharpnessOf(gray);
            string hint = format("sharpness=%.0f", sharp);
            Scalar hintColor(180, 180, 180);

            vector<Point> pts;
            bool detected = (sharp >= SHARP_MIN) &&
                            (qr.detect(gray, pts) && pts.size() == 4);

            if (sharp < SHARP_MIN)
            {
                hint += "  TOO BLURRY - hold steady";
                hintColor = Scalar(0, 100, 255);
            }
            else if (!detected)
            {
                hint += "  No QR in frame";
            }
            else
            {
                polylines(frame, pts, true, Scalar(0, 255, 255), 2);
                hint += "  QR locked, decoding...";
                hintColor = Scalar(0, 255, 255);

                // 维护锐度 top-3 候选缓存
                Cand cand{ sharp, pts, gray.clone() };
                cands.push_back(cand);
                sort(cands.begin(), cands.end(),
                     [](const Cand& a, const Cand& b) { return a.sharp > b.sharp; });
                if (cands.size() > 3) cands.pop_back();
            }

            // 解码节流：每 3 帧最多尝试一次，避免窗口卡顿
            if (frameNo - lastAttempt >= 3)
            {
                lastAttempt = frameNo;
                qrText = detected ? decodeFrameSmart(gray, qr) : "";
                if (qrText.empty() && !cands.empty() && !detected)
                    qrText = decodeFrameSmart(cands.front().gray, qr);   // 用历史最清晰的帧补刀
            }

            putText(frame, qrText.empty() ? hint : "QR decoded!",
                    Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.85,
                    qrText.empty() ? hintColor : Scalar(0, 255, 0), 2, LINE_AA);
            imshow("Camera (ESC to quit, s to save frame)", frame);

            if (!qrText.empty()) { cout << "二维码内容: " << qrText << "\n"; break; }
            int key = waitKey(20);
            if (key == 27) { cout << "手动退出，未识别到二维码\n"; return 4; }
            if (key == 's') {                               // 保存当前帧用于离线调试
                imwrite("last_frame.png", frame);
                cout << "当前帧已保存为 last_frame.png（可用 ./build/path_planner last_frame.png 测试）\n";
            }
        }
        destroyAllWindows();
    }

    cout << "\n===== 二维码文本 =====\n" << qrText << "\n======================\n";

    // ---- 解析 -> A* -> 绘图 ----
    GridMap map;
    if (!parseQRString(qrText, map)) { cerr << "二维码文本解析失败\n"; return 5; }

    cout << "地图 " << map.rows << "x" << map.cols
         << "  起点(行,列)=(" << map.start.y << "," << map.start.x << ")"
         << "  终点(行,列)=(" << map.goal.y << "," << map.goal.x << ")\n";

    AStarResult res = astar(map);
    if (res.found)
    {
        cout << "A* 找到最短路径！总代价 = " << res.totalCost
             << "，路径步数 = " << res.path.size() - 1 << "\n路径(行,列): ";
        for (const auto& p : res.path) cout << "(" << p.y << "," << p.x << ") ";
        cout << "\n";
    }
    else
    {
        cout << "A* 未找到可达路径（终点被障碍封锁）\n";
    }

    Mat vis = drawMap(map, res);
    imshow("A* Shortest Path", vis);
    imwrite("shortest_path.png", vis);                // 同时保存结果图
    cout << "结果图已保存为 shortest_path.png，按任意键退出\n";
    waitKey(0);
    return res.found ? 0 : 6;
}
