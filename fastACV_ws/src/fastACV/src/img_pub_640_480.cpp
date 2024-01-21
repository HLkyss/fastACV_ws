#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <dirent.h>

std::vector<std::string> getSortedFileNames(const std::string& directory) {
    std::vector<std::string> file_names;
    DIR* dir;
    struct dirent* ent;

    if ((dir = opendir(directory.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            std::string file_name = ent->d_name;
            // 忽略隐藏文件和目录
            if (file_name[0] != '.') {
                file_names.push_back(directory + "/" + file_name);
            }
        }
        closedir(dir);
    } else {
        // 无法打开目录
        perror("无法打开目录");
    }

    std::sort(file_names.begin(), file_names.end());
    return file_names;
}

cv::Mat increaseContrastInColorImage(const cv::Mat& input) {
    // 检查图像是否为空
    if (input.empty()) {
        throw std::runtime_error("输入图像为空");
    }

    // 将图像转换到 HSV 色彩空间
    cv::Mat hsv_image;
    cv::cvtColor(input, hsv_image, cv::COLOR_BGR2HSV);

    // 分离 HSV 通道
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv_image, hsv_channels);

    // 对 V 通道（亮度）进行直方图均衡化
    cv::equalizeHist(hsv_channels[2], hsv_channels[2]);

    // 合并通道
    cv::Mat contrast_enhanced_hsv;
    cv::merge(hsv_channels, contrast_enhanced_hsv);

    // 将图像转换回 BGR 色彩空间
    cv::Mat contrast_enhanced_bgr;
    cv::cvtColor(contrast_enhanced_hsv, contrast_enhanced_bgr, cv::COLOR_HSV2BGR);

    return contrast_enhanced_bgr;
}

cv::Mat resizeCropAndCompressImage(const cv::Mat& input, bool isLeftCamera, int target_width, int target_height, int size_limit_kb) {
    // 计算缩放比例 0.8888
//    float scale_width = target_width / static_cast<float>(input.cols);
//    float scale_height = target_height / static_cast<float>(input.rows);
//    float scale = std::max(scale_width, scale_height);
//    std::cout<<"scale:"<<scale<<std::endl;
    float scale =0.8;

    // 缩放图像
    cv::Mat scaled_image;
    cv::resize(input, scaled_image, cv::Size(), scale, scale, cv::INTER_LINEAR);
    std::cout<<"缩放图片大小："<<scaled_image.size()<<std::endl;

//    // 确保裁剪区域不超出缩放后图像的边界
//    int crop_x = std::max(0, (scaled_image.cols - target_width) / 2);
//    int crop_y = std::max(0, (scaled_image.rows - target_height) / 2);
//    std::cout<<"crop_x:"<<crop_x<<std::endl;
//    std::cout<<"crop_y:"<<crop_y<<std::endl;
//
//    if (isLeftCamera && scaled_image.cols > target_width) {
//        crop_x = 0; // 左相机：从左侧裁剪
//    } else if (!isLeftCamera && scaled_image.cols > target_width) {
//        crop_x = scaled_image.cols - target_width; // 右相机：从右侧裁剪
//    }
// 计算裁剪区域
    int crop_x = 0;
    int crop_y = 0;
    if (isLeftCamera) {
        // 左相机：保留右侧和下侧
        crop_x = std::max(0, scaled_image.cols - target_width);
        crop_y = std::max(0, scaled_image.rows - target_height);
    } else {
        // 右相机：保留左侧和下侧
        crop_y = std::max(0, scaled_image.rows - target_height);
    }

    cv::Rect crop_region(crop_x, crop_y, std::min(target_width, scaled_image.cols - crop_x), std::min(target_height, scaled_image.rows - crop_y));
    cv::Mat cropped_image = scaled_image(crop_region);
    std::cout<<"最终裁剪图片大小："<<cropped_image.size()<<std::endl;

    // 增加对比度
    cv::Mat contrast_enhanced_image;
    //! 线性调整
//    float alpha = -1.5f; // 对比度增益 1.5f
//    int beta = 0; // 亮度增益
//    cropped_image.convertTo(contrast_enhanced_image, -1, alpha, beta);
    //! 直方图均衡化
//    std::vector<cv::Mat> channels;
//    cv::split(cropped_image, channels);
//    for (int i = 0; i < 3; i++) {
//        cv::equalizeHist(channels[i], channels[i]);
//    }
//    cv::merge(channels, contrast_enhanced_image);
    //! Lab颜色空间
//    cv::Mat lab_image;
//    cv::cvtColor(cropped_image, lab_image, cv::COLOR_BGR2Lab);
//    std::vector<cv::Mat> lab_planes;
//    cv::split(lab_image, lab_planes);
//    cv::equalizeHist(lab_planes[0], lab_planes[0]); // 或使用CLAHE
//    cv::merge(lab_planes, lab_image);
//    cv::cvtColor(lab_image, contrast_enhanced_image, cv::COLOR_Lab2BGR);
    //! 在 HSV 色彩空间中增加图像对比度，防止颜色失真
    increaseContrastInColorImage(cropped_image).copyTo(contrast_enhanced_image);


    // 压缩图像
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY};
    int quality = 95; // 初始压缩质量
    std::vector<uchar> compressed_image;
    int compressed_size_kb;

    do {
        compressed_image.clear();
        compression_params.push_back(quality);
        cv::imencode(".jpg", contrast_enhanced_image, compressed_image, compression_params);
        compressed_size_kb = compressed_image.size() / 1024;

        quality -= 5; // 逐步降低质量
        if (quality < 10) break; // 防止质量过低
    } while (compressed_size_kb > size_limit_kb);

    if (compressed_size_kb > size_limit_kb) {
        ROS_ERROR("无法将图像压缩到指定大小以下");
        return cv::Mat();
    }

    return cv::imdecode(compressed_image, cv::IMREAD_COLOR);
}

// 新增函数：调整图像尺寸并进行压缩
cv::Mat resizeAndCompressImage(const cv::Mat& input, int target_width, int target_height, int size_limit_kb) {
    cv::Mat resized_image;
    cv::resize(input, resized_image, cv::Size(target_width, target_height));

    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY};
    int quality = 95; // 初始压缩质量
    std::vector<uchar> compressed_image;
    int compressed_size_kb;

    do {
        compressed_image.clear();
        compression_params.push_back(quality);
        cv::imencode(".jpg", resized_image, compressed_image, compression_params);
        compressed_size_kb = compressed_image.size() / 1024;

        quality -= 5; // 逐步降低质量
        if (quality < 10) break; // 防止质量过低
    } while (compressed_size_kb > size_limit_kb);

    if (compressed_size_kb > size_limit_kb) {
        ROS_ERROR("无法将图像压缩到指定大小以下");
        return cv::Mat();
    }

    return cv::imdecode(compressed_image, cv::IMREAD_COLOR);
}

cv::Mat compressImage(const cv::Mat& input, int size_limit_kb) {
    // 检查输入图像是否为空
    if (input.empty()) {
        ROS_ERROR("输入图像为空");
        return cv::Mat();
    }

    // 初始化压缩参数
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY};
    int quality = 95; // 初始压缩质量
    std::vector<uchar> compressed_image;
    int compressed_size_kb;

    do {
        compressed_image.clear();
        compression_params.push_back(quality);
        cv::imencode(".jpg", input, compressed_image, compression_params);
        compressed_size_kb = compressed_image.size() / 1024;

        quality -= 5; // 逐步降低质量
        if (quality < 10) break; // 防止质量过低
    } while (compressed_size_kb > size_limit_kb);

    if (compressed_size_kb > size_limit_kb) {
        ROS_ERROR("无法将图像压缩到指定大小以下");
        return cv::Mat();
    }

    return cv::imdecode(compressed_image, cv::IMREAD_COLOR);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    // 创建两个发布者，分别为两个话题
    ros::Publisher pub_image1 = nh.advertise<sensor_msgs::Image>("/camera/image_raw1", 1);
    ros::Publisher pub_image2 = nh.advertise<sensor_msgs::Image>("/camera/image_raw2", 1);

    // 设置循环频率
    ros::Rate loop_rate(5); // 5 Hz

    // 图片文件路径
//    std::string image_path1 = "/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/left0.jpg";
//    std::string image_path2 = "/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/right0.jpg";

    std::string folder_path1 = "/media/hl/Stuff/ubuntu_share_2/Dataset/ue_pin_fov100/theta0/cam0"; // 替换为实际路径
    std::string folder_path2 = "/media/hl/Stuff/ubuntu_share_2/Dataset/ue_pin_fov100/theta0/cam1"; // 替换为实际路径
    std::vector<std::string> file_names1 = getSortedFileNames(folder_path1);
    std::vector<std::string> file_names2 = getSortedFileNames(folder_path2);
    if (file_names1.size() != file_names2.size()) {
        ROS_ERROR("两个文件夹中的图像数量不一致！");
        return -1;
    }


    for (size_t i = 0; i < file_names1.size() && ros::ok(); ++i) {
        cv::Mat image1 = cv::imread(file_names1[i], cv::IMREAD_COLOR);
        cv::Mat image2 = cv::imread(file_names2[i], cv::IMREAD_COLOR);

        if (image1.empty() || image2.empty()) {
            ROS_ERROR("无法加载图片，请检查路径！");
            break;
        }

        //不调整图像尺寸，只压缩

        // 调整图像尺寸并压缩
//        cv::Mat resized_compressed_image1 = resizeAndCompressImage(image1, 640, 480, 1024);
//        cv::Mat resized_compressed_image2 = resizeAndCompressImage(image2, 640, 480, 1024);
        cv::Mat resized_compressed_image1 = resizeCropAndCompressImage(image1, true, 640, 480, 1024);
        cv::Mat resized_compressed_image2 = resizeCropAndCompressImage(image2, false, 640, 480, 1024);
        //只压缩图片大小
//        cv::Mat resized_compressed_image1 = compressImage(image1,  1024);
//        cv::Mat resized_compressed_image2 = compressImage(image2,  1024);

        if (resized_compressed_image1.empty() || resized_compressed_image2.empty()) {
            ROS_ERROR("图像压缩失败");
            continue;
        }

        // 将处理后的 OpenCV 图像转换为 ROS 图像消息
        sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_compressed_image1).toImageMsg();
        sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_compressed_image2).toImageMsg();

//        sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_compressed_image1).toImageMsg();
//        sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_compressed_image2).toImageMsg();

        // 发布图像消息
        pub_image1.publish(msg1);
        pub_image2.publish(msg2);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
