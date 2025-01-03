#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <cstdlib>
#include <cstdio>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <png.h>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <termios.h>
#include "ros/ros.h"
#include "jaka_msgs/Move.h"
#include "jaka_msgs/GetIK.h"
#include "jaka_msgs/SetIO.h"
#include "geometry_msgs/TwistStamped.h"
#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jktypes.h"

#define WIDTH 1920
#define HEIGHT 1080
#define PI 3.1415926
const float BLOCK_DIS = 31; // distance between neighbour block center (mm)
const float Z_WITH_CLEARANCE = 133.565; // (mm)
const float Z_WITHOUT_CLEARANCE = 115.800; // (mm)
const float Z_NORMAL = 139.565; // (mm)
const float INITIAL_JOINT[6] = {1.571, 0.598, 1.237, 0, 1.307, 0};
const float FINAL_JOINT[6] = {3.549*PI/180, 29.841*PI/180, 77.1*PI/180, 0, 73.419*PI/180, -86.451*PI/180};
const float LD[3] = {6.586, -281.378, Z_WITH_CLEARANCE}; // left down block center (mm) has z distance
const float LM[3] = {LD[0], LD[1]-BLOCK_DIS, LD[2]};
const float LU[3] = {LD[0], LD[1]-2*BLOCK_DIS, LD[2]};
const float MD[3] = {LD[0]-BLOCK_DIS, LD[1], LD[2]}; // middel down block center (mm)
const float MM[3] = {LD[0]-BLOCK_DIS, LD[1]-BLOCK_DIS, LD[2]};
const float MU[3] = {LD[0]-BLOCK_DIS, LD[1]-2*BLOCK_DIS, LD[2]};
const float RD[3] = {LD[0]-2*BLOCK_DIS, LD[1], LD[2]}; // up down block center (mm)
const float RM[3] = {LD[0]-2*BLOCK_DIS, LD[1]-BLOCK_DIS, LD[2]};
const float RU[3] = {LD[0]-2*BLOCK_DIS, LD[1]-2*BLOCK_DIS, LD[2]};

//g++ -std=c++17 -o capture_image capture_image.cpp -lv4l2 -lpng -lcurl -ljsoncpp

void set_camera_settings(int fd) {
    struct v4l2_control control;

    // 设置亮度
    control.id = V4L2_CID_BRIGHTNESS;
    control.value = 0.5; // 根据需求调整亮度值
    if (ioctl(fd, VIDIOC_S_CTRL, &control) == -1) {
        perror("Failed to set brightness");
    }

    // 设置增益
    control.id = V4L2_CID_GAIN;
    control.value = 0.5; // 根据需求调整增益值
    if (ioctl(fd, VIDIOC_S_CTRL, &control) == -1) {
        perror("Failed to set gain");
    }

    // 设置曝光模式为手动
    control.id = V4L2_CID_EXPOSURE_AUTO;
    control.value = V4L2_EXPOSURE_MANUAL;
    if (ioctl(fd, VIDIOC_S_CTRL, &control) == -1) {
        perror("Failed to set exposure mode");
    }

    // 设置曝光值
    control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    control.value = 50; // 根据需要调整曝光值
    if (ioctl(fd, VIDIOC_S_CTRL, &control) == -1) {
        perror("Failed to set exposure");
    }
}

// YUYV 转 RGB 格式
void yuyv_to_rgb(unsigned char* yuyv, unsigned char* rgb, int width, int height) {
    for (int i = 0; i < width * height; i += 2) {
        int y1 = yuyv[0];
        int u = yuyv[1] - 128;
        int y2 = yuyv[2];
        int v = yuyv[3] - 128;

        int r1 = y1 + 1.402 * v;
        int g1 = y1 - 0.344136 * u - 0.714136 * v;
        int b1 = y1 + 1.772 * u;

        int r2 = y2 + 1.402 * v;
        int g2 = y2 - 0.344136 * u - 0.714136 * v;
        int b2 = y2 + 1.772 * u;

        rgb[0] = std::clamp(r1, 0, 255);
        rgb[1] = std::clamp(g1, 0, 255);
        rgb[2] = std::clamp(b1, 0, 255);

        rgb[3] = std::clamp(r2, 0, 255);
        rgb[4] = std::clamp(g2, 0, 255);
        rgb[5] = std::clamp(b2, 0, 255);

        yuyv += 4;
        rgb += 6;
    }
}

// 保存 RGB 数据为 PNG 文件
void save_png(const char* filename, unsigned char* rgb_buffer, int width, int height) {
    FILE* fp = fopen(filename, "wb");
    if (!fp) {
        perror("Failed to open file for writing");
        exit(1);
    }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        fclose(fp);
        perror("Failed to create PNG write structure");
        exit(1);
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_write_struct(&png, nullptr);
        fclose(fp);
        perror("Failed to create PNG info structure");
        exit(1);
    }

    if (setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, &info);
        fclose(fp);
        perror("Failed during PNG creation");
        exit(1);
    }

    png_init_io(png, fp);

    // 设置 PNG 文件头信息
    png_set_IHDR(
        png,
        info,
        width,
        height,
        8,  // 每个通道 8 位
        PNG_COLOR_TYPE_RGB,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_DEFAULT,
        PNG_FILTER_TYPE_DEFAULT
    );
    png_write_info(png, info);

    // 写入图像数据
    png_bytep row_pointers[height];
    for (int i = 0; i < height; i++) {
        row_pointers[i] = rgb_buffer + (i * width * 3);
    }
    png_write_image(png, row_pointers);

    // 结束写入
    png_write_end(png, nullptr);
    png_destroy_write_struct(&png, &info);
    fclose(fp);
}

// 捕获图像并保存为 PNG 文件
int capture_image(const char* device, const char* output_filename) {
    int fd = open(device, O_RDWR);  // 打开摄像头设备
    if (fd == -1) {
        perror("Failed to open video device");
        return -1;
    }
    // 设置摄像头参数
    set_camera_settings(fd);
    // 查询摄像头支持的格式
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        perror("Failed to query capabilities");
        close(fd);
        return -1;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        std::cerr << "Device does not support video capture" << std::endl;
        close(fd);
        return -1;
    }

    // 设置摄像头格式为 YUYV
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("Failed to set format");
        close(fd);
        return -1;
    }

    // 请求缓冲区
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("Failed to request buffers");
        close(fd);
        return -1;
    }

    // 获取缓冲区地址
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;

    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
        perror("Failed to query buffer");
        close(fd);
        return -1;
    }

    void* buffer_start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    if (buffer_start == MAP_FAILED) {
        perror("Failed to mmap buffer");
        close(fd);
        return -1;
    }

    // 开始捕获
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        perror("Failed to start capture");
        close(fd);
        return -1;
    }

    // 捕获图像
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
        perror("Failed to queue buffer");
        close(fd);
        return -1;
    }

    if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
        perror("Failed to dequeue buffer");
        close(fd);
        return -1;
    }

    // 分配 RGB 数据缓冲区
    unsigned char* rgb_buffer = new unsigned char[WIDTH * HEIGHT * 3];
    yuyv_to_rgb((unsigned char*)buffer_start, rgb_buffer, WIDTH, HEIGHT);

    // 保存图像为 PNG 文件
    save_png(output_filename, rgb_buffer, WIDTH, HEIGHT);

    std::cout << "Image saved as " << output_filename << std::endl;

    // 释放资源
    delete[] rgb_buffer;

    if (ioctl(fd, VIDIOC_STREAMOFF, &type) == -1) {
        perror("Failed to stop capture");
        close(fd);
        return -1;
    }

    munmap(buffer_start, buf.length);
    close(fd);
    return 0;
}

// 读取文件内容
std::string read_file(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return "";
    }
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    return content;
}

// 回调函数，用于处理 CURL 返回的数据
size_t write_callback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    size_t total_size = size * nmemb;
    userp->append(static_cast<char*>(contents), total_size);
    return total_size;
}

// 调用 API 的函数
std::string call_api(const std::string& image_path, const std::string& token_file, const std::string& api_web_file) {
    // 读取 Token 和 API 地址
    std::string token = read_file(token_file);
    std::string api_web = read_file(api_web_file);

    // 去除可能的换行符
    token.erase(token.find_last_not_of("\n\r") + 1);
    api_web.erase(api_web.find_last_not_of("\n\r") + 1);

    if (token.empty() || api_web.empty()) {
        std::cerr << "Token or API address is empty!" << std::endl;
        return "";
    }

    CURL* curl;
    CURLcode res;
    std::string response_data;

    curl = curl_easy_init(); // 初始化 CURL
    if (curl) {
        struct curl_httppost* post = NULL;
        struct curl_httppost* last = NULL;

        // 添加图片文件到 POST 表单
        curl_formadd(&post, &last,
                     CURLFORM_COPYNAME, "images",
                     CURLFORM_FILE, image_path.c_str(),
                     CURLFORM_END);

        // 设置 CURL 选项
        curl_easy_setopt(curl, CURLOPT_URL, api_web.c_str()); // 设置 API 地址
        curl_easy_setopt(curl, CURLOPT_HTTPPOST, post);      // 设置 POST 表单

        // 添加自定义 Header，包括 Token
        struct curl_slist* headers = NULL;
        std::string auth_header = "X-Auth-Token:" + token;
        headers = curl_slist_append(headers, auth_header.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // 设置回调函数处理返回数据
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);

        // 执行请求
        res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        }

        // 清理
        curl_easy_cleanup(curl);
        curl_formfree(post);
        curl_slist_free_all(headers);
    }

    return response_data;
}

// 定义方块结构
struct Square {
    float x_center;
    float y_center;
};

// 解析 JSON 并计算中心坐标
std::vector<Square> parse_and_calculate(const std::string& api_response) {
    std::vector<Square> squares;
    Json::Value root;
    Json::CharReaderBuilder reader;
    std::string errs;

    // 解析 JSON 数据
    std::istringstream s(api_response);
    if (!Json::parseFromStream(reader, s, &root, &errs)) {
        std::cerr << "Failed to parse API response: " << errs << std::endl;
        return squares;
    }

     // 提取 detection_boxes 和 detection_scores
    const Json::Value boxes = root["detection_boxes"];
    const Json::Value scores = root["detection_scores"];

    // 检查 boxes 和 scores 的大小是否匹配
    if (boxes.size() != scores.size()) {
        std::cerr << "Mismatch between detection_boxes and detection_scores sizes" << std::endl;
        return squares;
    }

    for (Json::ArrayIndex i = 0; i < boxes.size(); ++i) {
        const Json::Value& box = boxes[i];
        float score = scores[i].asFloat();
        //std::cout<<score<<"\n";
        // 排除准确度小于 0.7 的方块
        if (score < 0.7) {
            continue;
        }

        if (box.size() == 4) {
            float ymin = box[0].asFloat();
            float xmin = box[1].asFloat();
            float ymax = box[2].asFloat();
            float xmax = box[3].asFloat();

            // 计算中心点
            float x_center = (xmin + xmax) / 2.0f;
            float y_center = (ymin + ymax) / 2.0f;

            squares.push_back({x_center, y_center});
        }
    }

    return squares;
}


void mouse_callback(int event, int x, int y, int, void* userdata) {
    // 从 userdata 中获取回调数据
    auto* callback_data = reinterpret_cast<std::pair<cv::Mat*, std::vector<cv::Point>*>*>(userdata);

    // 如果左键按下
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (callback_data->second->size() < 6) { // 限制为 6 个点
            callback_data->second->emplace_back(x, y);
            std::cout << "Point " << callback_data->second->size() << ": (" << x << ", " << y << ")" << std::endl;

            // 在图像上绘制点
            cv::circle(*callback_data->first, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
            cv::imshow("Image", *callback_data->first);

            if (callback_data->second->size() == 6) {
                std::cout << "You have selected 6 points. Exiting..." << std::endl;
            }
        }
    }
}

// 获取图像上的点坐标
std::vector<cv::Point> get_points_from_image(const std::string& image_path, int num_points = 6) {
    // 加载图像
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        return {};
    }

    // 用于存储选择的点
    std::vector<cv::Point> points;

    // 创建窗口
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

    // 设置鼠标回调函数
    std::pair<cv::Mat*, std::vector<cv::Point>*> callback_data = {&image, &points};
    cv::setMouseCallback("Image", mouse_callback, &callback_data);

    // 显示图像
    cv::imshow("Image", image);
    std::cout << "Please click on " << num_points << " points in the image." << std::endl;

    // 等待用户选择点
    while (points.size() < num_points) {
        int key = cv::waitKey(1);
        if (key == 27) { // 如果用户按下 ESC 键，退出程序
            std::cout << "User exited before selecting all points." << std::endl;
            break;
        }
    }

    // 销毁窗口
    cv::destroyAllWindows();

    return points;
}

// 获取机器人笛卡尔坐标信息
std::vector<double> getCartesianPosition(JAKAZuRobot& robot) {
    std::vector<double> cartesian_position(6, 0.0); // 返回的笛卡尔位姿数据 (X, Y, Z, Roll, Pitch, Yaw)

    // 获取机器人状态
    RobotStatus robot_status;
    int ret = robot.get_robot_status(&robot_status);
    if (ret != 0) {
        std::cerr << "Failed to get robot status, error code: " << ret << std::endl;
        return cartesian_position; // 返回默认值
    }

    // 保存笛卡尔坐标信息到返回值
    cartesian_position[0] = robot_status.cartesiantran_position[0]; // X
    cartesian_position[1] = robot_status.cartesiantran_position[1]; // Y
    cartesian_position[2] = robot_status.cartesiantran_position[2]; // Z
    cartesian_position[3] = robot_status.cartesiantran_position[3]; // Roll
    cartesian_position[4] = robot_status.cartesiantran_position[4]; // Pitch
    cartesian_position[5] = robot_status.cartesiantran_position[5]; // Yaw

    // 打印笛卡尔坐标信息
    std::cout << "Cartesian position (mm and radians):" << std::endl;
    std::cout << "X: " << cartesian_position[0] << " mm" << std::endl;
    std::cout << "Y: " << cartesian_position[1] << " mm" << std::endl;
    std::cout << "Z: " << cartesian_position[2] << " mm" << std::endl;
    std::cout << "Roll: " << cartesian_position[3] << " rad" << std::endl;
    std::cout << "Pitch: " << cartesian_position[4] << " rad" << std::endl;
    std::cout << "Yaw: " << cartesian_position[5] << " rad" << std::endl;

    return cartesian_position; // 返回位姿数据
}

cv::Mat computeTransformationMatrix(
    const std::vector<cv::Point2f>& pixel_points,
    const std::vector<cv::Point2f>& robot_positions) {
    // 检查点的数量是否匹配
    if (pixel_points.size() != robot_positions.size() || pixel_points.size() < 3) {
        std::cerr << "Error: At least 3 matching points are required, and sizes must match!" << std::endl;
        return cv::Mat();
    }

    // 计算仿射变换矩阵
    cv::Mat affine_matrix = cv::estimateAffine2D(pixel_points, robot_positions);

    // 检查矩阵是否有效
    if (affine_matrix.empty()) {
        std::cerr << "Error: Failed to compute affine transformation matrix!" << std::endl;
    }

    return affine_matrix;
}


// 将像素点转换为机器人坐标的函数
cv::Point2f pixelToRobot(const cv::Point& pixel_point, const cv::Mat& affine_matrix) {
    if (affine_matrix.empty()) {
        std::cerr << "Error: Affine transformation matrix is empty!" << std::endl;
        return cv::Point2f(0.0, 0.0);
    }

    // 应用仿射变换
    cv::Point2f robot_position;
    robot_position.x = affine_matrix.at<double>(0, 0) * pixel_point.x +
                       affine_matrix.at<double>(0, 1) * pixel_point.y +
                       affine_matrix.at<double>(0, 2);
    robot_position.y = affine_matrix.at<double>(1, 0) * pixel_point.x +
                       affine_matrix.at<double>(1, 1) * pixel_point.y +
                       affine_matrix.at<double>(1, 2);

    return robot_position;
}

// 等待按下 y 键
void wait_for_continue(bool is_output) {
    char input = '\0'; // 存储用户输入
    do {
        if(is_output){std::cout << "Press 'y' to continue..." << std::endl;}
        std::cin >> input; // 等待用户输入

        // 清空输入缓冲区，包括换行符
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    } while (input != 'y'); // 如果输入不是 'y'，继续提示
}

// 保存仿射变换矩阵和 z_average 到文件
void saveCalibrationData(const std::string& filename, const cv::Mat& affine_matrix, double z_average) {
    std::ofstream file(filename, std::ios::out);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file to save calibration data." << std::endl;
        return;
    }

    // 保存仿射变换矩阵
    file << "AffineMatrix:" << std::endl;
    for (int i = 0; i < affine_matrix.rows; ++i) {
        for (int j = 0; j < affine_matrix.cols; ++j) {
            file << affine_matrix.at<double>(i, j) << " ";
        }
        file << std::endl;
    }

    // 保存 Z 平均值
    file << "ZAverage:" << std::endl;
    file << z_average << std::endl;

    file.close();
    std::cout << "Calibration data saved successfully to " << filename << std::endl;
}

bool loadCalibrationData(const std::string& filename, cv::Mat& affine_matrix, double& z_average) {
    std::ifstream file(filename, std::ios::in);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file to load calibration data." << std::endl;
        return false;
    }

    std::cout << "Reading calibration data from: " << filename << std::endl;

    // 初始化仿射变换矩阵
    affine_matrix = cv::Mat(2, 3, CV_64F);
    std::string line;

    // 读取 AffineMatrix 标题
    if (std::getline(file, line)) {
        line.erase(line.find_last_not_of(" \t\r\n") + 1); // 移除行末空格
        std::cout << "Read line: " << line << std::endl; // 调试输出
    }
    if (line != "AffineMatrix:") {
        std::cerr << "Error: Expected 'AffineMatrix:', but got: " << line << std::endl;
        return false;
    }

    // 逐行读取仿射矩阵数据
    for (int i = 0; i < affine_matrix.rows; ++i) {
        for (int j = 0; j < affine_matrix.cols; ++j) {
            if (!(file >> affine_matrix.at<double>(i, j))) {
                std::cerr << "Error: Failed to read AffineMatrix value at row " << i << " col " << j << std::endl;
                return false;
            }
        }
    }
    // 跳过可能的空行
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // 读取 ZAverage 标题
    if (std::getline(file, line)) {
        line.erase(line.find_last_not_of(" \t\r\n") + 1); // 移除行末空格
        std::cout << "Read line: " << line << std::endl; // 调试输出
    }
    if (line != "ZAverage:") {
        std::cerr << "Error: Expected 'ZAverage:', but got: " << line << std::endl;
        return false;
    }

    // 读取 ZAverage 值
    if (!(file >> z_average)) {
        std::cerr << "Error: Failed to read ZAverage value." << std::endl;
        return false;
    }

    file.close();
    std::cout << "Calibration data loaded successfully." << std::endl;
    std::cout << "Affine Matrix:\n" << affine_matrix << std::endl;
    std::cout << "Z Average: " << z_average << std::endl;
    return true;
}

bool sucker(JAKAZuRobot& robot, bool flag){
  int ret = robot.set_digital_output(IOType::IO_TOOL, 1, flag);
  if (ret == 0) {
      ROS_INFO("Sucker %s",
               flag ? "off":"on");
      return true;
  } else {
      ROS_ERROR("Failed to %s sucker.",flag ? "off":"on");
      return false;
  }
}

bool suckerOn(JAKAZuRobot& robot){
  return sucker(robot, false);
}

bool suckerOff(JAKAZuRobot& robot){
  return sucker(robot, true);
}


bool move_to_xyz_with_fixed_orientation(JAKAZuRobot& robot, float x, float y, float z, float tolerance = 0.9, float speed = 650, float accel = 650, bool is_block = false) {
    //speed 移动速度 (单位：mm/s)
    //accel 加速度 (单位：mm/s²)
    //tolerance 容忍误差 (单位：mm)
    
    // 定义目标位置
    CartesianPose target_pose;

    // 设置目标位置的 x, y, z 坐标
    target_pose.tran.x = x;
    target_pose.tran.y = y;
    target_pose.tran.z = z;

    // 设置固定的 Roll, Pitch, Yaw
    target_pose.rpy.rx = -3.14;          // Roll
    target_pose.rpy.ry = 0.0;          // Pitch
    target_pose.rpy.rz = -3.14/ 2;       // Yaw

    OptionalCond *option_cond = nullptr;

    // 调用机器人线性移动接口
    int ret = robot.linear_move(&target_pose, MoveMode::ABS, is_block, speed, accel, tolerance, option_cond);

    // 检查执行结果并返回状态
    if (ret == 0) {
        ROS_INFO("Successfully moved to position: X=%.2f, Y=%.2f, Z=%.2f, Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
                 x, y, z, target_pose.rpy.rx, target_pose.rpy.ry, target_pose.rpy.rz);
        return true;
    } else {
        ROS_ERROR("Failed to move to position. ");
        return false;
    }
}

void wait_for_second(float sec){
  ros::WallDuration duration(sec);
  ros::spinOnce();
  duration.sleep();
}

bool pose_initialize(JAKAZuRobot& robot, const float* pose){
  JointValue joint_pos = {pose[0],pose[1],pose[2],
  			  pose[3],pose[4],pose[5]};
  int ret = robot.joint_move(&joint_pos, MoveMode::ABS, false, 3.14); // speed is 1rad/s
  if (ret == 0) {
      ROS_INFO("Pose initialization has done");
      return true;
  } else {
      ROS_ERROR("Pose initialization has failed");
      return false;
  }
}

bool get_sucker_state(JAKAZuRobot& robot){
    BOOL result=0;
    int ret = robot.get_digital_output(IOType::IO_TOOL, 1,&result);
    if (ret == 0) {
      ROS_INFO("get_sucker_state done");
  } else {
      ROS_ERROR("get_sucker_state failed");
  }
    return result;
}

bool judge_sucker_state(JAKAZuRobot& robot, bool target){
    return get_sucker_state(robot)==target;
}

bool move_from_xyz_to_xyz(JAKAZuRobot& robot, float begin_x, float begin_y, const float* block_xyz){
  int ret = robot.set_block_wait_timeout(0.51);
  if (ret == 0) {
      ROS_INFO("set_block_wait_timeout done");
  } else {
      ROS_ERROR("set_block_wait_timeout failed");
  }
  pose_initialize(robot,INITIAL_JOINT);
  move_to_xyz_with_fixed_orientation(robot, begin_x, begin_y, Z_NORMAL, 10.0);
  move_to_xyz_with_fixed_orientation(robot, begin_x, begin_y, Z_WITHOUT_CLEARANCE, 5.0);
  wait_for_second(0.1);
  move_to_xyz_with_fixed_orientation(robot, begin_x, begin_y, Z_NORMAL, 10.0);
  pose_initialize(robot,FINAL_JOINT);
  return true;
}


int main(int argc, char** argv) {
    const char* device = "/dev/video0";          // 摄像头设备路径
    const char* output_filename = "Calibration-image.png";  // 输出图像文件
    const std::string calibration_file = "calibration_data.txt";
    cv::Mat affine_matrix;
    double z_average;
    ros::init(argc, argv, "pick_place");
    ros::NodeHandle n;

    std::string robot_ip = "10.5.5.100"; // 机器人 IP
    JAKAZuRobot robot; // 初始化机器人对象
    std::vector<std::vector<double>> all_positions;

    // 连接机器人
    int ret = robot.login_in(robot_ip.c_str());
    if (ret != 0) {
        std::cerr << "Failed to connect to robot, error code: " << ret << std::endl;
        return -1;
    }
    std::cout << "Connected to robot at IP: " << robot_ip << std::endl;

    // 检查是否需要沿用上次标定结果
    char use_previous;
    std::cout << "Do you want to use the previous calibration result? (y/n): ";
    std::cin >> use_previous;

    if (use_previous == 'y' || use_previous == 'Y') {
        // 尝试加载上次标定结果
        if (!loadCalibrationData(calibration_file, affine_matrix, z_average)) {
            std::cerr << "Failed to load previous calibration data. Exiting..." << std::endl;
            return -1;
        }
    } else {
        
        if (capture_image(device, output_filename) == 0) {
            std::cout << "Capture successful!" << std::endl;
        } else {
            std::cerr << "Capture failed!" << std::endl;
        }

        // 调用函数获取用户选择的点
        int n = 3;
        std::vector<cv::Point> points = get_points_from_image(output_filename, n);
        // 输出选择的点
        if (points.empty()) {
            std::cerr << "No points selected or operation aborted." << std::endl;
        }
        
        double z_sum = 0.0; // 用于累计 Z 值
        z_average = 0.0; 
        // 获取 n 次位姿数据
        for (int i = 1; i <= n; ++i) {
            std::cout << "\nPress y to fetch Cartesian position (" << i << "/" << n << ")...";
            wait_for_continue(false);
            // 调用获取位姿的函数
            std::vector<double> cartesian_position = getCartesianPosition(robot);
            all_positions.push_back(cartesian_position);
            z_sum += cartesian_position[2];
        }
        // 计算 Z 值平均值
        if (n > 0) {
            z_average = z_sum / n;
        }
        
        std::vector<cv::Point2f> pixel_points;    // 像素点
        std::vector<cv::Point2f> robot_positions; // 机器人对应的 X、Y 坐标

        // 将像素点转换为 cv::Point2f
        for (const auto& point : points) {
            pixel_points.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
        }

        // 从 all_positions 中提取机器人坐标的 X 和 Y，并转换为 cv::Point2f
        for (const auto& position : all_positions) {
            robot_positions.emplace_back(static_cast<float>(position[0]), static_cast<float>(position[1]));
        }

        // 计算仿射变换矩阵
        affine_matrix = computeTransformationMatrix(pixel_points, robot_positions);
        saveCalibrationData(calibration_file, affine_matrix, z_average);
    }
    std::cout<<"Start obtaining block coordinates:\n ";
    wait_for_continue(true);
    const char* output_filename2 = "Objects-image.png";  // 输出图像文件
    if (capture_image(device, output_filename2) == 0) {
        std::cout << "Capture successful!" << std::endl;
    } else {
        std::cerr << "Capture failed!" << std::endl;
    }

    const std::string image_path = "Objects-image.png";      // 图片路径
    const std::string token_file = "X-Auth-Token.txt"; // Token 文件路径
    const std::string api_web_file = "api-web.txt";  // API 地址文件路径
    
    // 调用 API
    std::string response = call_api(image_path, token_file, api_web_file);

    // 输出完整的 API 回复
    if (!response.empty()) {
        std::cout << "Full API Respon`se:\n" << response << std::endl;
        // 解析 API 返回值并计算中心点
        std::vector<Square> squares = parse_and_calculate(response);

        // 遍历所有方块，依次移动
        for (size_t i = 0; i < squares.size(); ++i) {
            std::cout << "\nPress y to move to point (" << i + 1 << "/" << squares.size() << ")...";
            // wait_for_continue(false);
            // 将像素点中心坐标转换为机器人坐标
            cv::Point pixel_point(static_cast<int>(squares[i].x_center), static_cast<int>(squares[i].y_center));
            cv::Point2f robot_target = pixelToRobot(pixel_point, affine_matrix);

            std::cout<<robot_target.x<<" "<<robot_target.y;
           
            wait_for_continue(true);
             suckerOn(robot);

            move_from_xyz_to_xyz(robot,robot_target.x,robot_target.y,LD);
             wait_for_continue(true);
            suckerOff(robot);
        }
    } else {
        std::cerr << "Failed to get a valid response from the API." << std::endl;
    }
    return 0;
}

