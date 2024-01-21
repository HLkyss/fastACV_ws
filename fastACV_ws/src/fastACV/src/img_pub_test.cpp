#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    // 创建两个发布者，分别为两个话题
    ros::Publisher pub_image1 = nh.advertise<sensor_msgs::Image>("/camera/image_raw1", 1);
    ros::Publisher pub_image2 = nh.advertise<sensor_msgs::Image>("/camera/image_raw2", 1);

    // 设置循环频率
    ros::Rate loop_rate(5); // 5 Hz

    // 图片文件路径
    std::string image_path1 = "/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/left0.jpg";
    std::string image_path2 = "/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/right0.jpg";

    while (ros::ok()) {
        cv::Mat image1 = cv::imread(image_path1, cv::IMREAD_COLOR);
        cv::Mat image2 = cv::imread(image_path2, cv::IMREAD_COLOR);

        if (image1.empty() || image2.empty()) {
            ROS_ERROR("无法加载图片，请检查路径！");
            break;
        }

        // 将 OpenCV 图像转换为 ROS 图像消息
        sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image1).toImageMsg();
        sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image2).toImageMsg();

        // 发布图像消息
        pub_image1.publish(msg1);
        pub_image2.publish(msg2);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
