#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;
int main(){
    //1
    cv::Mat img=cv::imread("../assets/lena.jpg");
    if(img.empty()){
        std::cerr<<"Wrong"<<std::endl;
        return -1;
    }
    ////cv::imshow("lena",img);
    
    std::cout<<"rows="<<img.rows<<endl;
    cout<<"cols="<<img.cols<<endl;
    cout<<"channels="<<img.channels()<<endl;
    cout<<"step="<<img.step<<endl;
    cv::Rect rect(200,180,220,240);
    cv::Mat roi=img(rect).clone();
    cv::Mat img_copy=img.clone();
    // cv::rectangle(img_copy, rect, cv::Scalar(255, 255, 255), 2); 
    cv::rectangle(img_copy,rect,cv::Scalar(0,255,0),2);
    //imshow("img_copy",img_copy);
    //imshow("roi",roi);
    // cv::imwrite("", img);
    imwrite("../results/lena_marked.png",img_copy);
    imwrite("ROI.png",roi);
    //pixel clone
    cv::Mat roi_copy=roi.clone();
    cv::Vec3b& pixel1=roi_copy.at<cv::Vec3b>(0,0);
    cv::Vec3b& pixel2=roi.at<cv::Vec3b>(0,0);
    int blue=pixel1[0];
    int green=pixel1[1];
    int red=pixel1[2];
    cout<<blue<<" "<<green<<" "<<red<<endl;
    blue=pixel2[0];
    green=pixel2[1];
    red=pixel2[2];
    cout<<blue<<" "<<green<<" "<<red<<endl;
    //pixel change
    pixel1=cv::Vec3b(0,0,255);
    pixel2=cv::Vec3b(0,0,255);
    blue=pixel1[0];
    green=pixel1[1];
    red=pixel1[2];
    cout<<blue<<" "<<green<<" "<<red<<endl;
    blue=pixel2[0];
    green=pixel2[1];
    red=pixel2[2];
    cout<<blue<<" "<<green<<" "<<red<<endl;
    //imshow("ROI change",img);
    //imshow("ROI copy",roi_copy);

    cv::waitKey(0);
    cv::destroyAllWindows();
    
    //2
    cv::Mat img2=cv::imread("../assets/coins.png");
    if(img2.empty()){
        std::cerr<<"Wrong"<<std::endl;
        return -1;
    }
    ////cv::imshow("coins",img2);
    //turn gray
    cv::Mat gray_coins;

    cv::cvtColor(img2, gray_coins, cv::COLOR_BGR2GRAY);
    ////cv::imshow("gray coins",gray_coins);

    cv::Mat gray_coins_copy=gray_coins.clone();
    cv::Mat mask;
    cv::threshold(gray_coins_copy, mask, 135, 255, cv::THRESH_BINARY);
    ////cv::imshow("mask1",mask);

    //fushi pengzhang
    cv::Mat kernel = cv::getStructuringElement(MORPH_CROSS, cv::Size(3, 3));
    
    //开运算去噪（去小白点）
    //cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    //闭运算填孔（补小黑洞）
    //cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // for(int i=0;i<=2;i++){
    //     cv::dilate(mask, mask, kernel);
    // }
    // 膨胀 2 次
    cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 2);
    cv::erode(mask, mask, kernel);
    // for(int i=0;i<=10;i++){
    //     cv::dilate(mask, mask, kernel);
    //     cv::erode(mask, mask, kernel);
    // }
    // for(int i=0;i<=10;i++){
    //      cv::erode(mask, mask, kernel);
    // }


    //lunkuo
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat coins_copy=img2.clone();
    // 5. 遍历轮廓，画线 + 画框
    int total=0;
    for (size_t i = 0; i < contours.size(); i++) {
        // 过滤：面积太小忽略
        double area = cv::contourArea(contours[i]);
        if (area < 250 || area >4000) continue;
        // ---- 画对齐框 ----
        total++;
        cv::Rect rect = cv::boundingRect(contours[i]);
        cv::rectangle(coins_copy, rect,
                      cv::Scalar(0, 255, 0),     // green
                      1);                        // 线宽 2
        
        cv::putText(coins_copy, std::to_string(total),
                    cv::Point(rect.x +3, rect.y + 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(0, 0, 255), 1);
    }
    cout<<total<<endl;
    ////cv::imshow("mask2",mask);
    ////cv::imshow("mask3",coins_copy);
    cv::imwrite("../results/coins_counted.png",coins_copy);
    cv::imwrite("../results/coins_binary.png",mask);

    cv::Mat img3=cv::imread("../assets/math.png");
    if(img3.empty()){
        std::cerr<<"Wrong"<<std::endl;
        return -1;
    }

    // 2. 定义源点（按题目要求的 左上、右上、右下、左下 顺序）
    std::vector<cv::Point2f> math_pts;
    math_pts.push_back(cv::Point2f(74, 85));    // 左上
    math_pts.push_back(cv::Point2f(490, 70));   // 右上
    math_pts.push_back(cv::Point2f(520, 520));  // 右下
    math_pts.push_back(cv::Point2f(36, 516));   // 左下

    // 3. 定义目标点（450x450 的正方形区域，同样按左上、右上、右下、左下顺序）
    std::vector<cv::Point2f> square_pts;
    square_pts.push_back(cv::Point2f(0, 0));      // 左上
    square_pts.push_back(cv::Point2f(449, 0));    // 右上
    square_pts.push_back(cv::Point2f(449, 449));  // 右下
    square_pts.push_back(cv::Point2f(0, 449));    // 左下

    // 4. 在原图副本上标出四个源点，并编号
    cv::Mat math_copy = img3.clone();
    for (size_t i = 0; i < math_pts.size(); i++) {
        // 画实心红圆点（-1 表示填充）
        cv::circle(math_copy, math_pts[i], 8, cv::Scalar(0, 0, 255), -1);
        
        // 写编号 (1, 2, 3, 4)，位置稍微偏移一点，防止压在圆点上
        std::string label = std::to_string(i + 1);
        cv::putText(math_copy, label, 
                    cv::Point(math_pts[i].x + 15, math_pts[i].y - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 3);
    }

    // 5. 求透视变换矩阵
    cv::Mat M = cv::getPerspectiveTransform(math_pts, square_pts);

    // 6. 使用 warpPerspective 输出 450x450 的矫正图
    cv::Mat warped;
    cv::warpPerspective(img3, warped, M, cv::Size(450, 450));

    // 7. 显示标点原图和矫正结果
    //cv::imshow("Source with 4 Points", math_copy);
    //cv::imshow("Warped Result", warped);

    // // 8. 保存结果到 results 文件夹（确保有 .png 后缀，防止之前报错）
    // bool save_ok1 = cv::imwrite("../results/sudoku_points.png", math_copy);
    // bool save_ok2 = cv::imwrite("../results/sudoku_corrected.png", warped);

    // if (save_ok1 && save_ok2) {
    //     std::cout << "【成功】标点原图和矫正结果已保存到 results 文件夹！" << std::endl;
    // } else {
    //     std::cerr << "【错误】保存失败，请检查 results 文件夹是否存在！" << std::endl;
    // }

    // 1. 读图并转 HSV
    cv::Mat src = cv::imread("../assets/fruits.jpg");
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    // 2. 提取绿色
    cv::Mat mask_raw;
    cv::inRange(hsv, cv::Scalar(35, 43, 46), cv::Scalar(85, 255, 255), mask_raw);

    // 3. 3x3 开操作去噪
    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat mask_open;
    cv::morphologyEx(mask_raw, mask_open, cv::MORPH_OPEN, kernel2);

    // 4. 显示并保存（注意不要漏了后缀 .png）
    cv::imshow("Original", src);
    // //cv::imshow("Mask Raw", mask_raw);
    // //cv::imshow("Mask Open", mask_open);
    // ... 别忘了 cv::imwrite 保存
    cv::imwrite("../results/fruits_mask.png",mask_raw);
    cv::imwrite("../results/fruits_open.png",mask_open);

    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}


// 要求
// 1. 读取 coins.png，得到灰度图并进行二值化。
// 2. 使用 findContours() 找外轮廓，用轮廓面积筛选硬币候选，排除背景大块和小噪点。
// 3. 统计筛选后的轮廓数，在原图副本上为每枚候选画框并标号。
// 4. 显示并保存二值图与标号结果，输出硬币数量。
// 参数提示
// 基础部分允许直接使用以下已试跑的参数，不要求寻找一套新参数：
// 阈值 135；THRESH_BINARY；RETR_EXTERNAL；
// CHAIN_APPROX_SIMPLE；保留面积 250～4000 像素²
// 先二值化，再筛选外轮廓；效果不好可以选择形态学操作。


// #include <opencv2/opencv.hpp>
// #include <iostream>

// int main() {
//     // 读取图片（注意路径，按规范在 build 目录下运行，所以要加 ../）
//     cv::Mat img = cv::imread("../assets/lena.jpg");

//     if (img.empty()) {
//         std::cerr << "【错误】无法读取图片，请检查路径是否正确！" << std::endl;
//         return -1;
//     }

//     // ================= 第2题：输出属性 =================
//     std::cout << "===== 图像基本信息 =====" << std::endl;
    
//     // 1. 输出行数（高度）
//     std::cout << "行数 (rows): " << img.rows << " 像素" << std::endl;
    
//     // 2. 输出列数（宽度）
//     std::cout << "列数 (cols): " << img.cols << " 像素" << std::endl;
    
//     // 3. 输出通道数
//     std::cout << "通道数 (channels): " << img.channels() << std::endl;
    
//     // 4. 输出行步长（每一行在内存中占用的字节数）
//     std::cout << "行步长 (step): " << img.step << " 字节" << std::endl;
    
//     // 5. 说明通道顺序
//     std::cout << "\n===== 通道顺序说明 =====" << std::endl;
//     std::cout << "彩色图像的通道顺序是 BGR (蓝、绿、红)，而不是 RGB (红、绿、蓝)！" << std::endl;
//     std::cout << "这是 OpenCV 的默认读取顺序，在做颜色提取时一定要注意。" << std::endl;
//     // ===================================================

//     // 显示图片
//     //cv::imshow("Lena", img);
    
//     // 如果有保存要求，保留这行（注意路径也加了 ../）
//     // cv::imwrite("../results/lena_output.png", img);

//     cv::waitKey(0);
//     cv::destroyAllWindows();
//     return 0;
// }