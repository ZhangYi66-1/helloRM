/*
 * ============================================================================
 *  自动迫击炮全流程（4.2）—— 单文件工程 main.cpp
 * ============================================================================
 *
 *  串口指令（与已烧录的下位机一致，9600 8N1 + CRLF）：
 *      A  炮台匀速向左转（持续转，直到收到 P）
 *      D  炮台匀速向右转（持续转，直到收到 P）
 *      P  炮台停止
 *      G  开始供弹（PB14 高 / PB15 低 + TIM1 给速度）
 *      J  停止供弹
 *      F  摩擦轮满速 -> 开始发射
 *      H  摩擦轮停止 -> 结束发射
 *
 *  ---------------------------------------------------------------------------
 *  全自动流程（启动后无需任何按键）：
 *      ① 向右转 1 秒            -> D, 1000ms, P
 *      ② 识别 left / right      -> 识别成功那一帧存 result/detect_xxx.png
 *      ③ 按结果转向             -> left  : A, 1500ms, P
 *                                  right : A,  500ms, P
 *      ④ 供弹 + 发射            -> G 与 F 同时发，持续 20 秒
 *                                  20 秒后 J 停供弹、H 停摩擦轮
 *      ⑤ 发射后复位转           -> left  : A,  500ms, P
 *                                  right : A, 1500ms, P
 *      ⑥ 扫描二维码 + A* 规划   -> 结果图存 result/path_xxx.png
 *      ⑦ 向右转 1 秒            -> D, 1000ms, P
 *  ---------------------------------------------------------------------------
 *
 *  输出文件（都放在程序工作目录下的 result/ 里，目录不存在会自动创建）：
 *      result/detect_<时间戳>.png     识别出方向的完成帧（绿框 + 方向标注）
 *      result/path_<时间戳>.png       A* 最短路径结果图
 *
 *  编译：见 CMakeLists.txt（目标名 school_competition_code）
 *  运行：./start
 * ============================================================================
 */

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#ifdef HAVE_ZBAR
#include <zbar.h>
#endif

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

/* ---- POSIX 相关 ---- */
#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <ctime>

using namespace cv;
using namespace std;

/* ============================ 串口封装 ============================ */
class SimpleSerial
{
public:
    SimpleSerial() {}
    ~SimpleSerial() { closePort(); }

    bool openPort(const string& dev, int baud = 9600, bool raiseDtr = true)
    {
        closePort();
        fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0) return false;

        termios tio;
        memset(&tio, 0, sizeof(tio));
        if (tcgetattr(fd_, &tio) != 0) { closePort(); return false; }

        cfmakeraw(&tio);
        speed_t sp = toSpeed(baud);
        cfsetispeed(&tio, sp);
        cfsetospeed(&tio, sp);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 5;

        if (tcsetattr(fd_, TCSANOW, &tio) != 0) { closePort(); return false; }
        tcflush(fd_, TCIOFLUSH);

        if (raiseDtr)
        {
            int bits = TIOCM_DTR | TIOCM_RTS;
            ::ioctl(fd_, TIOCMBIS, &bits);
        }
        return true;
    }

    /* 发一行，自动补 \r\n */
    bool writeLine(const string& line)
    {
        if (fd_ < 0) return false;
        string s = line + "\r\n";
        ssize_t n = ::write(fd_, s.data(), s.size());
        tcdrain(fd_);
        return n == (ssize_t)s.size();
    }

    void closePort() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
    bool isOpen() const { return fd_ >= 0; }

private:
    int fd_ = -1;
    static speed_t toSpeed(int baud)
    {
        switch (baud)
        {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B9600;
        }
    }
};

/* ============================ 可调参数 ============================ */
static const int    TARGET_FPS        = 30;      /* 识别帧率上限 */
static const int    SERIAL_BAUD       = 9600;    /* ★与串口助手一致，勿随意改 */
static const bool   SERIAL_RAISE_DTR  = true;

static const char*  CMD_LEFT     = "D";          /* 匀速左转 */
static const char*  CMD_RIGHT    = "A";          /* 匀速右转 */
static const char*  CMD_STOP     = "P";          /* 炮台停止 */
static const char*  CMD_FEED_ON  = "G";          /* 开始供弹 */
static const char*  CMD_FEED_OFF = "J";          /* 停止供弹 */
static const char*  CMD_FIRE_ON  = "F";          /* 摩擦轮满速 = 开始发射 */
static const char*  CMD_FIRE_OFF = "H";          /* 摩擦轮停止 = 结束发射 */

static const int    TURN_TO_RIGHT_MS   = 2000;   /* ① 先向右转 2 秒 */
static const int    TURN_LEFT_LONG_MS  = 2800;   /* 识别为 left 时左转 2.8 秒 */
static const int    TURN_LEFT_SHORT_MS = 1100;    /* 识别为 right 时左转 1.1 秒 */
static const int    TURN_AFTER_QR_MS   = 1200;    /* ⑦ 扫完码向右转 1.2 秒 */

static const int    FIRE_DURATION_MS = 20000;    /* 供弹 + 发射同时持续 20 秒 */

static const int    RESULT_SHOW_MS    = 1200;    /* 结果图停留时间 */

/* 识别参数（阈值已按实际环境调好，勿随意改动） */
static const double MIN_PAPER_AREA    = 10000;
static const double COLOR_DIFF_THRESH = 60.0;
static const int    MIN_SHAPE_AREA    = 300;
static const int    FRAME_CONFIRM     = 2;
static const bool   DIR_INVERT        = false;   /* 相机镜像 / 炮管装反时改 true */
static const bool   DIR_FINAL_INVERT  = true;    /* ★识别结束后 left/right 全部取反（含颜色规则之后） */

/* ============================ 通用工具 ============================ */
static void sleepMs(int ms) { this_thread::sleep_for(chrono::milliseconds(ms)); }

static int64_t nowMs()
{
    return (int64_t)chrono::duration_cast<chrono::milliseconds>(
               chrono::steady_clock::now().time_since_epoch()).count();
}

static void ensureResultDir() { ::mkdir("result", 0777); }   /* 已存在则忽略 */

static string resultPath(const char* prefix)
{
    char ts[64];
    time_t t = time(nullptr);
    struct tm lt;
    localtime_r(&t, &lt);
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &lt);

    char full[192];
    snprintf(full, sizeof(full), "result/%s_%s.png", prefix, ts);
    return string(full);
}

static void saveImage(const Mat& img, const char* prefix, const char* desc)
{
    if (img.empty()) { cout << "[存图] " << desc << " 为空，跳过" << endl; return; }
    string p = resultPath(prefix);
    if (imwrite(p, img)) cout << "[存图] " << desc << " -> " << p << endl;
    else                 cout << "[存图] 保存失败: " << p << endl;
}

/* ============================ 摄像头 ============================ */
static VideoCapture openCamera()
{
    DIR* dir = opendir("/dev");
    if (dir)
    {
        bool found = false;
        struct dirent* ent;
        cout << "[信息] /dev 下的视频设备: ";
        while ((ent = readdir(dir)) != nullptr)
        {
            string name = ent->d_name;
            if (name.rfind("video", 0) == 0) { cout << "/dev/" << name << " "; found = true; }
        }
        cout << (found ? "" : "（未找到任何 video 设备！）") << endl;
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

    cerr << "[错误] 索引 0~9 都打不开摄像头，检查 ls /dev/video* 与权限" << endl;
    return VideoCapture();
}

/* 关闭后重新打开并设成 720p：V4L2 在推流中途改分辨率经常不生效或只改一半，
   必须在第一次 read 之前把分辨率设好（二维码识别依赖高清帧） */
static VideoCapture reopenCamera720p()
{
    for (int idx = 0; idx < 10; ++idx)
    {
        /* 与原版完全一致：不指定后端（CAP_ANY），由 OpenCV 自动协商 */
        VideoCapture cap(idx);
        if (cap.isOpened())
        {
            /* 720p 未压缩格式受 USB 带宽限制只有几 fps，先请求 MJPG
               （摄像头不支持时该设置被自动忽略，无副作用） */
            cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap.set(CAP_PROP_FRAME_WIDTH,  1280);
            cap.set(CAP_PROP_FRAME_HEIGHT, 720);
            cap.set(CAP_PROP_AUTOFOCUS, 1);
            cout << "[信息] 二维码阶段重开摄像头 idx=" << idx
                 << "  实际分辨率 " << (int)cap.get(CAP_PROP_FRAME_WIDTH)
                 << "x" << (int)cap.get(CAP_PROP_FRAME_HEIGHT)
                 << "  帧率 " << cap.get(CAP_PROP_FPS) << " fps" << endl;
            return cap;
        }
    }
    return VideoCapture();
}

/* ============================ 图案识别 ============================ */
static double vertexAngle(const Point2f& prev, const Point2f& cur, const Point2f& next)
{
    Point2f v1 = prev - cur, v2 = next - cur;
    double l1 = norm(v1), l2 = norm(v2);
    if (l1 < 1e-6 || l2 < 1e-6) return 180.0;
    double cosA = v1.dot(v2) / (l1 * l2);
    cosA = max(-1.0, min(1.0, cosA));
    return acos(cosA) * 180.0 / CV_PI;
}

static int indexOfLargestAngle(const vector<Point>& contour)
{
    int n = (int)contour.size(), best = 0;
    double bestAngle = -1;
    for (int i = 0; i < n; ++i)
    {
        double a = vertexAngle(contour[(i - 1 + n) % n], contour[i], contour[(i + 1) % n]);
        if (a > bestAngle) { bestAngle = a; best = i; }
    }
    return best;
}

static Scalar meanColorOf(const Mat& bgr, const vector<Point>& contour)
{
    Mat mask = Mat::zeros(bgr.size(), CV_8UC1);
    fillPoly(mask, vector<vector<Point>>{contour}, Scalar(255));
    Mat eroded;
    erode(mask, eroded, getStructuringElement(MORPH_RECT, Size(5, 5)));
    return mean(bgr, (countNonZero(eroded) > 0) ? eroded : mask);
}

/* 0=三角形 1=四边形 2=圆形 -1=其他 */
static int classifyShape(const vector<Point>& approx, double area, double perimeter)
{
    double circularity = 4.0 * CV_PI * area / (perimeter * perimeter + 1e-6);
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
    Rect box;
};

/* 像二维码一样：先锁定黑底 -> 透视拉正成标准方块 -> 在拉正图里找三个图形
   -> 轮廓映射回原帧（方向几何 / 取色 / 标注都在原帧上做） */
static DetectedShapes detectShapes(const Mat& frame)
{
    DetectedShapes out;

    /* V = max(B,G,R)，拆通道 + 两次 cv::max，比整帧 BGR2HSV 快数倍 */
    Mat ch[3];
    split(frame, ch);
    Mat v;
    cv::max(ch[0], ch[1], v);
    cv::max(v, ch[2], v);
    GaussianBlur(v, v, Size(5, 5), 0);

    /* 宽于 640 的帧先等比缩小再算，阈值按面积比折算 */
    const double s = (frame.cols > 640) ? 640.0 / frame.cols : 1.0;
    const double areaScale = s * s;
    static const Mat openKernel = getStructuringElement(MORPH_RECT, Size(3, 3));

    /* ---- 1. 锁定黑底：面积 / 宽高比 / 实心度都合理的最大暗色块 ----
       ★面积下限自适应：只要求占画面 0.2%，远近通吃（固定 10000 像素在
       640 宽、1.5 米距离时黑底只有约 9500，每帧都被拒 → 永远 hit:0）。
       衣服裤子等背景由宽高比 + 实心度筛掉。暗阈值 70 不行再试 90。 */
    vector<Point> baseContour;
    double baseArea = 0;
    const double frameArea = (double)v.rows * v.cols;
    const double minBase = 0.002 * frameArea;
    for (int darkThr : { 70, 90 })
    {
        Mat dark;
        threshold(v, dark, darkThr, 255, THRESH_BINARY_INV);
        morphologyEx(dark, dark, MORPH_OPEN, openKernel);
        vector<vector<Point>> contours;
        findContours(dark, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        for (size_t i = 0; i < contours.size(); ++i)
        {
            const double a = contourArea(contours[i]);
            if (a < minBase || a > 0.35 * frameArea) continue;   // 太小 / 太大都不要
            const Rect r = boundingRect(contours[i]);
            const double ar = (double)r.width / (double)std::max(r.height, 1);
            if (ar < 0.35 || ar > 2.8) continue;                 // 细长条（裤腿等）不要
            const RotatedRect rr = minAreaRect(contours[i]);
            const double fill = a / ((double)rr.size.width * rr.size.height + 1e-6);
            if (fill < 0.6) continue;                            // 黑底是实心方块，镂空的不要
            if (a > baseArea) { baseArea = a; baseContour = contours[i]; }
        }
        if (!baseContour.empty()) break;                         // 70 档已找到就不用试 90
    }
    if (baseContour.empty()) return out;

    /* ---- 2. 黑底四角 -> 透视拉正成标准方块 ---- */
    vector<Point2f> quad;
    {
        vector<Point> approx;
        approxPolyDP(baseContour, approx,
                     0.02 * arcLength(baseContour, true), true);
        if ((int)approx.size() == 4)
            quad.assign(approx.begin(), approx.end());
        else
        {
            Point2f p4[4];
            minAreaRect(baseContour).points(p4);                 // 兜底：旋转矩形四角
            quad.assign(p4, p4 + 4);
        }
        Point2f cen(0, 0);
        for (const auto& p : quad) cen += p;
        cen.x /= 4.0f; cen.y /= 4.0f;
        sort(quad.begin(), quad.end(), [&cen](const Point2f& a, const Point2f& b) {
            return atan2(a.y - cen.y, a.x - cen.x) < atan2(b.y - cen.y, b.x - cen.x);
        });
    }
    auto sideLen = [](const Point2f& a, const Point2f& b) { return (float)norm(a - b); };
    const float baseSide = std::max({ sideLen(quad[0], quad[1]), sideLen(quad[1], quad[2]),
                                       sideLen(quad[2], quad[3]), sideLen(quad[3], quad[0]) });
    const int S = std::min(std::max((int)baseSide, 300), 800);   // 拉正后方块边长
    const int margin = std::max(S / 20, 4);                      // 目标框内缩，避免白色纸边混入
    /* 注意：getPerspectiveTransform 的两个参数必须同为 vector 或同为数组，
       混用 vector + C 数组会因重载不匹配而编译失败 */
    const vector<Point2f> dstQuad = { { (float)margin,       (float)margin },
                                      { (float)(S - margin), (float)margin },
                                      { (float)(S - margin), (float)(S - margin) },
                                      { (float)margin,       (float)(S - margin) } };
    const Mat H = getPerspectiveTransform(quad, dstQuad);

    Mat rectV;
    warpPerspective(v, rectV, H, Size(S, S), INTER_CUBIC);

    /* ---- 3. 在拉正的黑底内找亮图形（视角/倾斜已归一，背景已被隔离） ----
       ★Otsu 自适应阈值替代固定 90：拉正图里只有"黑底 + 亮图形"两档灰度，
       Otsu 永远能把它们分开，光线强弱都不怕 */
    Mat bright;
    threshold(rectV, bright, 0, 255, THRESH_BINARY | THRESH_OTSU);
    morphologyEx(bright, bright, MORPH_OPEN, openKernel);
    vector<vector<Point>> contours;
    findContours(bright, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    /* 拉正图中的面积阈值 = 原阈值 × s² × (S²/baseArea)（拉正的面积放大比） */
    const double minShapeRect =
        MIN_SHAPE_AREA * areaScale * (double)S * (double)S / (baseArea + 1e-6);

    double triArea = 0, sqArea = 0, cirArea = 0;
    vector<Point> triR, sqR, cirR;                               // 拉正坐标系下的轮廓
    for (size_t i = 0; i < contours.size(); ++i)
    {
        const double area = contourArea(contours[i]);
        if (area < minShapeRect) continue;

        const double peri = arcLength(contours[i], true);
        vector<Point> approx;
        approxPolyDP(contours[i], approx, 0.02 * peri, true);

        const int shape = classifyShape(approx, area, peri);
        if (shape == 0 && area > triArea) { triArea = area; triR = contours[i]; }
        if (shape == 1 && area > sqArea)  { sqArea  = area; sqR  = contours[i]; }
        if (shape == 2 && area > cirArea) { cirArea = area; cirR = contours[i]; }
    }

    /* ---- 4. 轮廓映射回原帧坐标 ---- */
    const Mat Hinv = H.inv();
    auto mapBack = [&Hinv](const vector<Point>& c) {
        if (c.empty()) return vector<Point>{};
        vector<Point2f> src, warped;
        src.reserve(c.size());
        for (const auto& p : c) src.emplace_back((float)p.x, (float)p.y);
        perspectiveTransform(src, warped, Hinv);
        vector<Point> outP;
        outP.reserve(warped.size());
        for (const auto& p : warped) outP.emplace_back(cvRound(p.x), cvRound(p.y));
        return outP;
    };
    out.tri = mapBack(triR);
    out.sq  = mapBack(sqR);
    out.cir = mapBack(cirR);

    /* 若处理时缩放过，再放大回原图分辨率 */
    if (s < 1.0)
    {
        const double inv = 1.0 / s;
        auto upscale = [inv](vector<Point>& c) {
            for (auto& p : c) { p.x = cvRound(p.x * inv); p.y = cvRound(p.y * inv); }
        };
        upscale(out.tri); upscale(out.sq); upscale(out.cir);
    }

    out.valid = !out.tri.empty() && !out.sq.empty() && !out.cir.empty();

    /* ★保险②：三图形聚拢检查（拉正后正常情况必然通过，仅作防异常兜底） */
    if (out.valid)
    {
        const Rect bt = boundingRect(out.tri), bs = boundingRect(out.sq),
                   bc = boundingRect(out.cir);
        const double maxDim = std::max({ bt.width, bt.height,
                                         bs.width, bs.height,
                                         bc.width, bc.height });
        const Point2f ct((bt.x + bt.width  / 2.0f), (bt.y + bt.height  / 2.0f));
        const Point2f cs((bs.x + bs.width  / 2.0f), (bs.y + bs.height  / 2.0f));
        const Point2f cc((bc.x + bc.width  / 2.0f), (bc.y + bc.height  / 2.0f));
        const double maxDist = std::max({ norm(ct - cs), norm(ct - cc), norm(cs - cc) });
        if (maxDist > 2.5 * std::max(maxDim, 1.0)) out.valid = false;
    }

    if (out.valid)
        out.box = boundingRect(out.tri) | boundingRect(out.sq) | boundingRect(out.cir);
    return out;
}

/* 判定：圆形 vs 正方形颜色；返回 left / right，并在图上标注 */
static string judge(Mat& frame, const DetectedShapes& s)
{
    Scalar cirColor = meanColorOf(frame, s.cir);
    Scalar sqColor  = meanColorOf(frame, s.sq);
    double colorDiff = norm(cirColor - sqColor);
    bool sameColor = (colorDiff < COLOR_DIFF_THRESH);

    int maxIdx = indexOfLargestAngle(s.tri);
    Point2f maxVertex(s.tri[maxIdx]);
    Moments m = moments(s.tri);
    Point2f centroid((float)(m.m10 / m.m00), (float)(m.m01 / m.m00));

    string direction = (maxVertex.x > centroid.x) ? "right" : "left";
    if (DIR_INVERT) direction = (direction == "right") ? "left" : "right";
    if (!sameColor) direction = (direction == "right") ? "left" : "right";

    /* ★最终结果全部取反：几何判定 + 颜色规则都算完之后，left <-> right 整体翻转 */
    if (DIR_FINAL_INVERT)
    {
        const string raw = direction;
        direction = (direction == "right") ? "left" : "right";
        cout << "[取反] " << raw << " -> " << direction << endl;
    }

    cout << "[判定] 圆形BGR=(" << cirColor[2] << "," << cirColor[1] << "," << cirColor[0]
         << ") 正方形BGR=(" << sqColor[2] << "," << sqColor[1] << "," << sqColor[0]
         << ") 色差=" << colorDiff
         << " 颜色" << (sameColor ? "相同" : "不同")
         << " | 最大角指向: " << direction << endl;

    /* 标注：绿框 + 红点 + 方向线 */
    rectangle(frame, s.box, Scalar(0, 255, 0), 3);
    circle(frame, maxVertex, 8, Scalar(0, 0, 255), -1);
    line(frame, centroid, maxVertex, Scalar(0, 0, 255), 2);
    putText(frame, direction, centroid + Point2f(-40, -40),
            FONT_HERSHEY_SIMPLEX, 1.2, Scalar(0, 0, 255), 3);
    putText(frame, sameColor ? "SAME COLOR" : "DIFF COLOR",
            Point(12, 34), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);
    return direction;
}

/* 识别阶段：连续命中 FRAME_CONFIRM 帧判定成功；成功帧存图 */
static bool detectDirection(VideoCapture& cap, string& directionOut, Mat& annotatedOut)
{
    const int64_t t0 = nowMs();
    const double frameIntervalMs = 1000.0 / TARGET_FPS;
    int64_t lastFrame = 0;
    int hitCount = 0;

    /* 不设超时：没识别到就一直识别（ESC 手动退出） */
    while (true)
    {
        if (nowMs() - lastFrame < frameIntervalMs) { sleepMs(2); continue; }

        Mat frame;
        if (!cap.read(frame) || frame.empty()) { sleepMs(5); continue; }
        lastFrame = nowMs();

        DetectedShapes cur = detectShapes(frame);
        if (cur.valid) ++hitCount; else hitCount = 0;

        if (cur.valid && hitCount >= FRAME_CONFIRM)
        {
            directionOut = judge(frame, cur);
            annotatedOut = frame.clone();
            saveImage(annotatedOut, "detect", "识别完成帧");
            destroyWindow("Mortar - Detect");
            return true;
        }

        /* 实时预览 */
        char info[128];
        snprintf(info, sizeof(info), "DETECTING  hit:%d  %.1fs",
                 hitCount, (nowMs() - t0) / 1000.0);
        putText(frame, info, Point(12, 34), FONT_HERSHEY_SIMPLEX, 0.75,
                Scalar(0, 255, 255), 2);
        if (cur.valid) rectangle(frame, cur.box, Scalar(0, 255, 0), 3);
        imshow("Mortar - Detect", frame);
        if (waitKey(1) == 27) break;
    }
    return false;
}

/* ============================ 二维码解码 ============================ */
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

static Mat unsharp(const Mat& gray)
{
    Mat blur, out;
    GaussianBlur(gray, blur, Size(0, 0), 2.0);
    addWeighted(gray, 1.6, blur, -0.6, 0, out);
    return out;
}

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

/* ============================ 二维码文本 -> 地图 ============================ */
static bool isBlocked(int v) { return v == 0; }

struct GridMap
{
    int rows = 0, cols = 0;
    Point start, goal;
    vector<vector<int>> cost;
};

static bool parseQRString(const string& s, GridMap& m)
{
    size_t pos = s.find(':');
    if (pos == string::npos) return false;

    stringstream head(s.substr(0, pos));
    string tok;
    vector<int> hv;
    while (getline(head, tok, ','))
    {
        tok.erase(remove_if(tok.begin(), tok.end(), ::isspace), tok.end());
        if (!tok.empty()) hv.push_back(atoi(tok.c_str()));
    }
    if (hv.size() < 6) return false;
    m.rows = hv[0]; m.cols = hv[1];
    m.start = Point(hv[3], hv[2]);
    m.goal  = Point(hv[5], hv[4]);
    if (m.rows <= 0 || m.cols <= 0) return false;

    vector<int> flat;
    stringstream body(s.substr(pos + 1));
    while (body >> tok)
    {
        size_t dash = tok.find('-');
        if (dash == string::npos) return false;
        int v = atoi(tok.substr(0, dash).c_str());
        int n = atoi(tok.substr(dash + 1).c_str());
        for (int i = 0; i < n; ++i) flat.push_back(v);
    }
    if ((int)flat.size() < m.rows * m.cols) return false;

    m.cost.assign(m.rows, vector<int>(m.cols, 0));
    int k = 0;
    for (int r = 0; r < m.rows; ++r)
        for (int c = 0; c < m.cols; ++c)
            m.cost[r][c] = flat[k++];
    return true;
}

/* ============================ A* 最短路径 ============================ */
struct AStarNode
{
    int r, c, g, f;
    bool operator>(const AStarNode& o) const { return f > o.f; }
};
struct AStarResult
{
    bool found = false;
    int totalCost = 0;
    vector<Point> path;
};

static AStarResult astar(const GridMap& m)
{
    AStarResult res;
    const int R = m.rows, C = m.cols;
    auto inside = [&](int r, int c) { return r >= 0 && r < R && c >= 0 && c < C; };
    auto walk   = [&](int r, int c) { return inside(r, c) && !isBlocked(m.cost[r][c]); };

    if (!walk(m.start.y, m.start.x) || !walk(m.goal.y, m.goal.x)) return res;

    int minCost = 1 << 30;
    for (int r = 0; r < R; ++r)
        for (int c = 0; c < C; ++c)
            if (!isBlocked(m.cost[r][c])) minCost = min(minCost, m.cost[r][c]);
    auto h = [&](int r, int c) {
        return (abs(r - m.goal.y) + abs(c - m.goal.x)) * minCost;
    };

    priority_queue<AStarNode, vector<AStarNode>, greater<AStarNode>> open;
    vector<vector<int>> gbest(R, vector<int>(C, 1 << 30));
    vector<vector<pair<int,int>>> parent(R, vector<pair<int,int>>(C, {-1, -1}));
    vector<vector<bool>> closed(R, vector<bool>(C, false));

    int sr = m.start.y, sc = m.start.x, gr = m.goal.y, gc = m.goal.x;
    gbest[sr][sc] = 0;
    open.push({sr, sc, 0, h(sr, sc)});

    const int dr[] = { -1, 1, 0, 0 };
    const int dc[] = {  0, 0,-1, 1 };

    while (!open.empty())
    {
        AStarNode cur = open.top(); open.pop();
        if (closed[cur.r][cur.c]) continue;
        closed[cur.r][cur.c] = true;

        if (cur.r == gr && cur.c == gc)
        {
            res.found = true;
            res.totalCost = gbest[gr][gc];
            pair<int,int> p = { gr, gc };
            while (!(p.first == sr && p.second == sc))
            {
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
            int ng = gbest[cur.r][cur.c] + m.cost[nr][nc];
            if (ng < gbest[nr][nc])
            {
                gbest[nr][nc] = ng;
                parent[nr][nc] = { cur.r, cur.c };
                open.push({ nr, nc, ng, ng + h(nr, nc) });
            }
        }
    }
    return res;
}

static Mat drawMap(const GridMap& m, const AStarResult& res)
{
    const int CELL = 80, MARGIN = 40;
    Mat img(m.rows * CELL + 2 * MARGIN, m.cols * CELL + 2 * MARGIN, CV_8UC3,
            Scalar(245, 245, 245));

    for (int r = 0; r < m.rows; ++r)
        for (int c = 0; c < m.cols; ++c)
        {
            int x = MARGIN + c * CELL, y = MARGIN + r * CELL;
            Scalar fill = isBlocked(m.cost[r][c]) ? Scalar(40, 40, 40) : Scalar(255, 255, 255);
            rectangle(img, Rect(x, y, CELL, CELL), fill, -1);
            rectangle(img, Rect(x, y, CELL, CELL), Scalar(120, 120, 120), 1);
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", m.cost[r][c]);
            putText(img, buf, Point(x + CELL / 2 - 8, y + CELL / 2 + 8),
                    FONT_HERSHEY_SIMPLEX, 0.55,
                    isBlocked(m.cost[r][c]) ? Scalar(255, 255, 255) : Scalar(90, 90, 90),
                    1, LINE_AA);
        }

    if (res.found)
        for (size_t i = 1; i < res.path.size(); ++i)
            line(img,
                 Point(MARGIN + res.path[i-1].x * CELL + CELL/2,
                       MARGIN + res.path[i-1].y * CELL + CELL/2),
                 Point(MARGIN + res.path[i].x   * CELL + CELL/2,
                       MARGIN + res.path[i].y   * CELL + CELL/2),
                 Scalar(0, 0, 255), 4, LINE_AA);

    auto center = [&](Point p) {
        return Point(MARGIN + p.x * CELL + CELL / 2, MARGIN + p.y * CELL + CELL / 2);
    };
    circle(img, center(m.start), 18, Scalar(0, 180, 0), -1, LINE_AA);
    putText(img, "S", center(m.start) + Point(-8, 7), FONT_HERSHEY_SIMPLEX,
            0.7, Scalar(255, 255, 255), 2, LINE_AA);
    circle(img, center(m.goal), 18, Scalar(255, 120, 0), -1, LINE_AA);
    putText(img, "E", center(m.goal) + Point(-8, 7), FONT_HERSHEY_SIMPLEX,
            0.7, Scalar(255, 255, 255), 2, LINE_AA);

    string info = res.found
        ? format("A* path cost = %d  steps = %d", res.totalCost, (int)res.path.size() - 1)
        : "A* path NOT found";
    putText(img, info, Point(MARGIN, 28), FONT_HERSHEY_SIMPLEX, 0.7,
            res.found ? Scalar(0, 0, 200) : Scalar(0, 0, 255), 2, LINE_AA);
    return img;
}

/* ===================================================================
 *  五、二维码识别阶段
 *  ------------------------------------------------------------------
 *  以下整段【原版照搬】，唯一的改动是 cap 由参数传入 —— 因为摄像头
 *  已经在迫击炮流程里打开了，不能再 new 一个 VideoCapture。
 * =================================================================== */
static bool qrStage(VideoCapture& cap, string& qrText)
{
    /* 关闭重开，保证 1280x720 真正生效（原版是在推流中途 set，V4L2 下经常无效） */
    cap.release();
    cap = reopenCamera720p();
    if (!cap.isOpened())
    {
        cerr << "[错误] 二维码阶段无法重新打开摄像头" << endl;
        return false;
    }
    /* 给自动对焦约 1 秒收敛：读帧驱动 AF，预热帧直接丢弃；顺带实测帧率 */
    {
        Mat warm;
        int got = 0;
        const int64_t tWarm = nowMs();
        while (nowMs() - tWarm < 1000) { if (cap.read(warm)) ++got; }
        cout << "[信息] 预热 1 秒读到 " << got << " 帧"
             << (got < 15 ? "（帧率偏低，二维码解码节奏会变慢）" : "") << endl;
    }
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
        if (key == 27) { cout << "手动退出，未识别到二维码\n"; destroyAllWindows(); return false; }
        if (key == 's') {                               // 保存当前帧用于离线调试
            imwrite("last_frame.png", frame);
            cout << "当前帧已保存为 last_frame.png（可用 ./build/path_planner last_frame.png 测试）\n";
        }
    }
    destroyAllWindows();
    return !qrText.empty();
}

/* ============================ 运动控制 ============================ */
static void sendCmd(SimpleSerial& s, const char* cmd, const char* desc)
{
    cout << "[串口] " << desc << " -> \"" << cmd << "\"" << endl;
    if (!s.isOpen()) { cout << "        串口未打开，跳过" << endl; return; }
    if (!s.writeLine(cmd)) cout << "        发送失败" << endl;
}

/* 转 ms 毫秒然后停：发方向 -> 等 -> 发 P */
static void turnFor(SimpleSerial& s, const char* dir, int ms, const char* desc)
{
    cout << "[动作] " << desc << "（" << ms << " ms）" << endl;
    sendCmd(s, dir, desc);
    sleepMs(ms);
    sendCmd(s, CMD_STOP, "停止");
}

/* ============================ 主流程 ============================ */
int main(int argc, char** argv)
{
    string motorDev = "/dev/ttyUSB0";
    for (int i = 1; i < argc; ++i)
    {
        string a = argv[i];
        if (!a.empty() && a[0] == '/') motorDev = a;
    }

    ensureResultDir();

    cout << "==================================================" << endl;
    cout << " 自动迫击炮全流程" << endl;
    cout << "==================================================" << endl;

    /* ---- 串口 ---- */
    SimpleSerial motor;
    if (motor.openPort(motorDev, SERIAL_BAUD, SERIAL_RAISE_DTR))
        cout << "[信息] 串口已打开: " << motorDev << " @" << SERIAL_BAUD << endl;
    else
        cout << "[警告] 串口打开失败: " << motorDev << "（后续动作只打印，不下发）" << endl;

    /* ---- 摄像头 ---- */
    VideoCapture cap = openCamera();
    if (!cap.isOpened()) return -1;
    cap.set(CAP_PROP_FPS, TARGET_FPS);

    /* ================= ① 向右转 1 秒 ================= */
    cout << "\n--- ① 向右转 1 秒 ---" << endl;
    turnFor(motor, CMD_RIGHT, TURN_TO_RIGHT_MS, "向右转");

    /* ================= ② 识别 left / right ================= */
    cout << "\n--- ② 识别 left / right ---" << endl;
    string direction;
    Mat detectFrame;
    if (!detectDirection(cap, direction, detectFrame))
    {
        cerr << "[提示] 手动退出（ESC），未得到 left/right，流程终止" << endl;
        cap.release();
        return 2;
    }
    cout << "[结果] 识别方向 = " << direction << endl;

    const bool isLeft = (direction == "left");

    /* ================= ③ 按结果转向 ================= */
    cout << "\n--- ③ 按识别结果转向 ---" << endl;
    const int aimMs = isLeft ? TURN_LEFT_LONG_MS : TURN_LEFT_SHORT_MS;
    turnFor(motor, CMD_LEFT, aimMs,
            isLeft ? "识别 left，向左转 1.5 秒" : "识别 right，向左转 0.5 秒");

    /* ============ ④ 同时开始供弹与发射，持续 20 秒 ============ */
    cout << "\n--- ④ 供弹 + 发射，持续 " << FIRE_DURATION_MS / 1000 << " 秒 ---" << endl;
    sendCmd(motor, CMD_FEED_ON, "开始供弹");
    sendCmd(motor, CMD_FIRE_ON, "开始发射（摩擦轮满速）");

    /* 20 秒持续过程中每 5 秒打印一次进度，方便盯着看 */
    for (int elapsed = 0; elapsed < FIRE_DURATION_MS; elapsed += 1000)
    {
        sleepMs(1000);
        if ((elapsed + 1000) % 5000 == 0)
            cout << "        已发射 " << (elapsed + 1000) / 1000 << " 秒 / "
                 << FIRE_DURATION_MS / 1000 << " 秒" << endl;
    }

    /* 20 秒到：停供弹、停摩擦轮 */
    sendCmd(motor, CMD_FEED_OFF, "停止供弹");
    sendCmd(motor, CMD_FIRE_OFF, "停止发射（摩擦轮停）");

    /* ================= ⑤ 发射后复位转向 ================= */
    cout << "\n--- ⑤ 发射后复位转向 ---" << endl;
    const int backMs = isLeft ? TURN_LEFT_SHORT_MS : TURN_LEFT_LONG_MS;
    turnFor(motor, CMD_LEFT, backMs,
            isLeft ? "识别 left，向左转 0.5 秒" : "识别 right，向左转 1.5 秒");

    /* ================= ⑥ 扫描二维码 + A* ================= */
    cout << "\n--- ⑥ 扫描二维码 ---" << endl;
    string qrText;
    qrStage(cap, qrText);

    cout << "\n===== 二维码文本 =====\n" << qrText << "\n======================\n";

    // ---- 解析 -> A* -> 绘图 ----
    GridMap map;
    if (!parseQRString(qrText, map))
    {
        cerr << "二维码文本解析失败\n";
    }
    else
    {
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
        saveImage(vis, "path", "A* 结果图");   // 原版是 imwrite("shortest_path.png", vis)
        waitKey(RESULT_SHOW_MS);               // 原版是 waitKey(0)，这里不等按键
    }

    /* ================= ⑦ 向右转 1 秒后停止 ================= */
    cout << "\n--- ⑦ 向右转 1 秒 ---" << endl;
    turnFor(motor, CMD_RIGHT, TURN_AFTER_QR_MS, "向右转");

    /* ================= 结束 ================= */
    cout << "\n==================================================" << endl;
    cout << " 流程结束" << endl;
    cout << "   识别方向 : " << direction << endl;
    cout << "   存图目录 : result/" << endl;
    cout << "==================================================" << endl;

    cap.release();
    destroyAllWindows();
    return 0;
}
