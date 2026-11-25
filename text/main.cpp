/*
 * A4 纸图案识别程序（OpenCV / C++）
 *
 * 场景：2.9 米正前方有一张 A4 纸，纸上印有图案（见 右01.png）：
 *   - 黑色底，上面有：一个三角形（左）、一个圆形（右上）、一个正方形（右下）
 *
 * 功能（两阶段）：
 *   阶段一【识别确认】：检测到 A4 纸上的三个图形（三角形/圆形/正方形）后，
 *                      用绿色框把它们框起来，等待用户按 'c' 确认；
 *   阶段二【输出判断】：用户确认后进行判断：
 *     1. 判断"圆形"和"正方形"的颜色是否相同
 *     2. 颜色相同 -> 输出三角形最大角的指向（left/right）
 *        颜色不同 -> 输出三角形最大角指向的反方向
 *     按 'r' 可返回阶段一重新识别
 *
 * 帧率：限制为 30 FPS
 *
 * 编译（Linux / macOS）：
 *   g++ detect_pattern.cpp -o detect_pattern `pkg-config --cflags --libs opencv4`
 * 编译（Windows + MinGW）：
 *   g++ detect_pattern.cpp -o detect_pattern.exe -I"C:/opencv/include" -L"C:/opencv/x64/mingw/lib" -lopencv_world4xx
 */

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <string>
#include <iostream>
#include <algorithm>
#include <dirent.h>

using namespace cv;
using namespace std;

// ---------------- 参数配置 ----------------
static const int    TARGET_FPS        = 30;     // 目标帧率
static const double MIN_PAPER_AREA     = 10000;   // A4 纸最小面积（像素），按分辨率调整
static const double COLOR_DIFF_THRESH  = 60.0;   // 判断两形状颜色是否相同的 BGR 欧氏距离阈值
static const int    MIN_SHAPE_AREA    = 300;    // 图案最小面积（2.9米处图形较小，阈值放低）

// ---------------- 状态机 ----------------
enum AppState
{
    STATE_WAIT_CONFIRM,  // 阶段一：已识别，画绿框等待确认
    STATE_JUDGED         // 阶段二：已完成判断，显示结果
};

/*
 * 摄像头打开（嵌入式板子健壮版）：
 * 1. 扫描 /dev/video* 列出所有视频设备
 * 2. 依次尝试索引 0~9（很多板子的摄像头不是 video0，常见 video11/video1 等）
 * 3. 全部失败则打印诊断信息
 */
static VideoCapture openCamera()
{
    DIR* dir = opendir("/dev");
    if (dir)
    {
        cout << "[信息] /dev 下的视频设备节点:" << endl;
        struct dirent* ent;
        bool found = false;
        while ((ent = readdir(dir)) != nullptr)
        {
            string name = ent->d_name;
            if (name.rfind("video", 0) == 0)
            {
                cout << "    /dev/" << name << endl;
                found = true;
            }
        }
        if (!found)
            cout << "    （未找到任何 video 设备！摄像头未连接或驱动未加载）" << endl;
        closedir(dir);
    }

    for (int idx = 0; idx < 10; ++idx)
    {
        VideoCapture cap(idx, CAP_V4L2);
        if (cap.isOpened())
        {
            cout << "[信息] 成功打开摄像头索引: " << idx << endl;
            return cap;
        }
    }

    cerr << "\n[错误] 所有索引 0~9 均无法打开摄像头，请按以下步骤排查:" << endl;
    cerr << "  1. ls /dev/video*          —— 确认设备节点是否存在" << endl;
    cerr << "  2. lsusb                   —— USB 摄像头是否被识别" << endl;
    cerr << "  3. dmesg | grep -iE 'uvc|camera|video'  —— 内核是否识别到摄像头" << endl;
    cerr << "  4. MIPI 摄像头需检查设备树 overlay 是否使能" << endl;
    cerr << "  5. 权限: sudo chmod 666 /dev/video* 或 sudo usermod -aG video $USER 后重新登录" << endl;

    return VideoCapture();
}

// 计算多边形某个顶点的内角（角度制）
static double vertexAngle(const Point2f& prev, const Point2f& cur, const Point2f& next)
{
    Point2f v1 = prev - cur;
    Point2f v2 = next - cur;
    double l1 = norm(v1), l2 = norm(v2);
    if (l1 < 1e-6 || l2 < 1e-6) return 180.0;
    double cosA = v1.dot(v2) / (l1 * l2);
    cosA = std::max(-1.0, std::min(1.0, cosA));
    return acos(cosA) * 180.0 / CV_PI;
}

// 根据顶点求各内角，返回最大角所在顶点下标
static int indexOfLargestAngle(const vector<Point>& contour)
{
    int n = (int)contour.size();
    int best = 0;
    double bestAngle = -1;
    for (int i = 0; i < n; ++i)
    {
        double a = vertexAngle(contour[(i - 1 + n) % n], contour[i], contour[(i + 1) % n]);
        if (a > bestAngle) { bestAngle = a; best = i; }
    }
    return best;
}

// 计算轮廓内部平均颜色（BGR），掩码收缩一圈避免边缘混色
static Scalar meanColorOf(const Mat& bgr, const vector<Point>& contour)
{
    Mat mask = Mat::zeros(bgr.size(), CV_8UC1);
    fillPoly(mask, vector<vector<Point>>{contour}, Scalar(255));
    Mat eroded;
    erode(mask, eroded, getStructuringElement(MORPH_RECT, Size(5, 5)));
    return mean(bgr, (countNonZero(eroded) > 0) ? eroded : mask);
}

// 形状分类：0=三角形, 1=四边形(正方形), 2=圆形, -1=其他
static int classifyShape(const vector<Point>& approx, double area, double perimeter)
{
    double circularity = 4.0 * CV_PI * area / (perimeter * perimeter + 1e-6);
    // 圆形优先判断（圆度最高），放宽阈值以适应远距离低分辨率
    if (circularity > 0.82 && (int)approx.size() >= 5) return 2;
    if ((int)approx.size() == 3) return 0;
    if ((int)approx.size() == 4 || (int)approx.size() == 5)
    {
        Rect r = boundingRect(approx);
        double fill = area / ((double)r.width * r.height + 1e-6);
        if (fill > 0.75) return 1;
    }
    return -1;
}

struct DetectedShapes
{
    bool valid = false;
    vector<Point> tri, sq, cir;
    Rect box; // 三个图形的联合外接矩形（绿框）
};

/*
 * 在帧中定位 A4 纸并提取三个图形。
 * 关键逻辑（自内向外）：
 *   1. （可选）定位白色 A4 纸
 *   2. 在纸面内找"黑色图案底"——V 通道暗区的最大轮廓
 *   3. 在黑底范围内找"亮色图形"（白色三角形、蓝色圆形、蓝色正方形）——
 *      它们是黑底上的高亮度区域，而不是黑色区域本身！
 */
static DetectedShapes detectShapes(const Mat& frame)
{
    DetectedShapes out;

    Mat hsv, v;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    // 提取 V（亮度）通道
    Mat ch[3];
    split(hsv, ch);
    v = ch[2];
    Mat blurred;
    GaussianBlur(v, blurred, Size(5, 5), 0);

    // ---- 1. 定位 A4 纸（最大白色四边形，失败则降级为全图）----
    Mat paperMask = Mat::zeros(frame.size(), CV_8UC1);
    {
        Mat bin;
        threshold(blurred, bin, 160, 255, THRESH_BINARY); // 白纸，阈值放宽
        vector<vector<Point>> contours;
        findContours(bin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        double bestArea = 0;
        int bestIdx = -1;
        for (size_t i = 0; i < contours.size(); ++i)
        {
            double a = contourArea(contours[i]);
            if (a > MIN_PAPER_AREA && a > bestArea)
            {
                vector<Point> approx;
                approxPolyDP(contours[i], approx, 0.02 * arcLength(contours[i], true), true);
                if ((int)approx.size() >= 4 && (int)approx.size() <= 6)
                { bestArea = a; bestIdx = (int)i; }
            }
        }
        if (bestIdx >= 0)
            drawContours(paperMask, contours, bestIdx, Scalar(255), -1);
        else
            paperMask.setTo(Scalar(255));
    }

    // ---- 2. 找黑色图案底（V 暗区最大轮廓）----
    Mat dark;
    Rect patternROI;
    threshold(blurred, dark, 70, 255, THRESH_BINARY_INV);
    bitwise_and(dark, paperMask, dark);
    morphologyEx(dark, dark, MORPH_OPEN,
                 getStructuringElement(MORPH_RECT, Size(3, 3)));

    {
        vector<vector<Point>> contours;
        findContours(dark, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        double bestArea = 0;
        int bestIdx = -1;
        for (size_t i = 0; i < contours.size(); ++i)
        {
            double a = contourArea(contours[i]);
            if (a > MIN_PAPER_AREA && a > bestArea) { bestArea = a; bestIdx = (int)i; }
        }
        if (bestIdx < 0) return out; // 没有黑底 -> 图案不完整
        patternROI = boundingRect(contours[bestIdx]);
    }

    // ---- 3. 在黑底内提取亮色图形 ----
    Mat bright;
    threshold(blurred, bright, 90, 255, THRESH_BINARY); // 白三角/蓝图形亮度都远高于黑底
    Mat roiMask = Mat::zeros(frame.size(), CV_8UC1);
    rectangle(roiMask, patternROI, Scalar(255), -1);
    bitwise_and(bright, roiMask, bright);
    morphologyEx(bright, bright, MORPH_OPEN,
                 getStructuringElement(MORPH_RECT, Size(3, 3)));

    vector<vector<Point>> contours;
    findContours(bright, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    double triArea = 0, sqArea = 0, cirArea = 0;
    for (size_t i = 0; i < contours.size(); ++i)
    {
        double area = contourArea(contours[i]);
        if (area < MIN_SHAPE_AREA) continue;

        double peri = arcLength(contours[i], true);
        vector<Point> approx;
        approxPolyDP(contours[i], approx, 0.02 * peri, true);

        int shape = classifyShape(approx, area, peri);
        if (shape == 0 && area > triArea)  { triArea = area;  out.tri  = contours[i]; }
        if (shape == 1 && area > sqArea)   { sqArea = area;   out.sq   = contours[i]; }
        if (shape == 2 && area > cirArea)  { cirArea = area;  out.cir  = contours[i]; }
    }

    // 三个图形都齐了才算有效识别
    out.valid = !out.tri.empty() && !out.sq.empty() && !out.cir.empty();
    if (out.valid)
    {
        Rect r1 = boundingRect(out.tri);
        Rect r2 = boundingRect(out.sq);
        Rect r3 = boundingRect(out.cir);
        out.box = r1 | r2 | r3;
    }
    return out;
}

// 阶段二：执行判断，返回结果字符串
static string judge(const Mat& frame, const DetectedShapes& s)
{
    // 1. 比较"圆形"和"正方形"的颜色
    Scalar cirColor = meanColorOf(frame, s.cir);
    Scalar sqColor  = meanColorOf(frame, s.sq);
    double colorDiff = norm(cirColor - sqColor); // BGR 欧氏距离
    bool sameColor = (colorDiff < COLOR_DIFF_THRESH);

    // 2. 求三角形最大角指向
    int maxIdx = indexOfLargestAngle(s.tri);
    Point2f maxVertex(s.tri[maxIdx]);
    Moments m = moments(s.tri);
    Point2f centroid((float)(m.m10 / m.m00), (float)(m.m01 / m.m00));
    string direction = (maxVertex.x > centroid.x) ? "left" : "right";
    if (!sameColor) // 颜色不同 -> 输出反方向
        direction = (direction == "left") ? "right" : "left";

    char buf[512];
    snprintf(buf, sizeof(buf),
             "圆形BGR=(%.0f,%.0f,%.0f) 正方形BGR=(%.0f,%.0f,%.0f) 色差=%.1f 颜色%s | 三角形最大角指向: %s",
             cirColor[2], cirColor[1], cirColor[0],
             sqColor[2],  sqColor[1],  sqColor[0],
             colorDiff,
             sameColor ? "相同" : "不同",
             direction.c_str());

    // 可视化：红点标最大角顶点，标注方向
    circle(const_cast<Mat&>(frame), maxVertex, 6, Scalar(0, 0, 255), -1);
    line(const_cast<Mat&>(frame), centroid, maxVertex, Scalar(0, 0, 255), 2);
    putText(const_cast<Mat&>(frame), direction, centroid + Point2f(-30, -30),
            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);

    return string(buf);
}

int main()
{
    VideoCapture cap = openCamera();
    if (!cap.isOpened()) return -1;

    // 降低帧率到 30 FPS
    cap.set(CAP_PROP_FPS, TARGET_FPS);
    cout << "[信息] 摄像头帧率设置为: " << cap.get(CAP_PROP_FPS) << " FPS" << endl;

    const double frameIntervalMs = 1000.0 / TARGET_FPS;
    auto lastTime = getTickCount();

    AppState state = STATE_WAIT_CONFIRM;
    DetectedShapes detected;
    string result;
    Mat frozen;          // 暂停时显示的冻结帧
    bool paused = false; // 检测到图案后暂停画面，等用户按键

    namedWindow("A4 Pattern Detection", WINDOW_NORMAL);

    bool quit = false;
    while (!quit)
    {
        Mat frame;

        if (!paused)
        {
            // ================= 实时识别 =================
            // ---- 30FPS 节流 ----
            double elapsedMs = (getTickCount() - lastTime) * 1000.0 / getTickFrequency();
            if (elapsedMs < frameIntervalMs) continue;

            if (!cap.read(frame) || frame.empty())
            {
                cerr << "[错误] 读取帧失败" << endl;
                break;
            }
            lastTime = getTickCount();

            DetectedShapes cur = detectShapes(frame);
            if (cur.valid)
            {
                // 识别成功 -> 冻结当前画面，暂停等待按键
                detected = cur;
                frozen = frame.clone();
                paused = true;
                cout << "[信息] 已识别到图案，画面已暂停。"
                        "按 [c] 确认并判断，按 [x] 放弃本次并继续实时识别" << endl;
                continue;
            }

            putText(frame, "Searching pattern...", Point(10, 30),
                    FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
            imshow("A4 Pattern Detection", frame);
            int key = waitKey(1);
            if (key == 27) break;
        }
        else
        {
            // ================= 画面暂停，等待按键 =================
            frame = frozen.clone();
            rectangle(frame, detected.box, Scalar(0, 255, 0), 3);

            if (state == STATE_WAIT_CONFIRM)
            {
                putText(frame, "Paused  [c]=OK  [x]=skip", Point(10, 30),
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
            }
            else // STATE_JUDGED
            {
                putText(frame, "[r]=re-detect  [ESC]=quit", Point(10, 30),
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                putText(frame, result.substr(0, 70), Point(10, frame.rows - 20),
                        FONT_HERSHEY_SIMPLEX, 0.55, Scalar(0, 255, 0), 2);
            }

            imshow("A4 Pattern Detection", frame);
            int key = waitKey(30); // 暂停期间降低轮询频率
            if (key == 27) break;

            if (key == 'c' && state == STATE_WAIT_CONFIRM)
            {
                // 用户确认 -> 在冻结帧上执行判断
                result = judge(frozen, detected);
                state = STATE_JUDGED;
                cout << "\n[结果] " << result << endl;
            }
            else if (key == 'x' && state == STATE_WAIT_CONFIRM)
            {
                paused = false; // 放弃本次，恢复实时画面
            }
            else if (key == 'r' && state == STATE_JUDGED)
            {
                // 返回实时识别
                state = STATE_WAIT_CONFIRM;
                paused = false;
                result.clear();
                cout << "[信息] 返回实时识别..." << endl;
            }
        }
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
