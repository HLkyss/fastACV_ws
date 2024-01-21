#include"FastACVNet_plus_Algorithm.h"
#include<iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include<chrono>

ros::Publisher image_publisher1;//您的ROS节点似乎没有发布/processed_image1话题的原因可能是因为发布器image_publisher1是在回调函数imageCallback中声明和初始化的。在每次回调执行时，都会创建一个新的发布器，但由于作用域的限制，这个发布器在回调函数结束时会被销毁。因此，可能导致消息没有被成功发布到ROS网络。解决这个问题的一个方法是将发布器声明为全局变量或将其作为节点句柄的成员。这样，发布器在整个节点的生命周期内都是活动的，可以确保消息的发布。
// 函数声明
cv::Mat heatmap(cv::Mat &disparity);
void imageCallback(const sensor_msgs::ImageConstPtr &msg1, const sensor_msgs::ImageConstPtr &msg2);

cv::Mat heatmap(cv::Mat&disparity)
{
    //max min
    cv::Mat image_re = disparity.reshape(1);
    double minValue, maxValue;   
    cv::Point  minIdx, maxIdx;     
    cv::minMaxLoc(image_re, &minValue, &maxValue, &minIdx, &maxIdx);
    
    cv::Mat mean_mat(cv::Size(disparity.cols,disparity.rows), CV_32FC1, minValue);
    cv::Mat std_mat(cv::Size(disparity.cols,disparity.rows), CV_32FC1, (maxValue-minValue)/255);

    cv::Mat norm_disparity_map = (disparity - mean_mat) / std_mat;
    cv::Mat heap_map,abs_map;
    cv::convertScaleAbs(norm_disparity_map,abs_map,1);
    cv::applyColorMap(abs_map,heap_map,cv::COLORMAP_COOL);  //COLORMAP_JET
//    COLORMAP_AUTUMN
//    COLORMAP_BONE
//    COLORMAP_JET
//    COLORMAP_WINTER
//    COLORMAP_RAINBOW
//    COLORMAP_OCEAN
//    COLORMAP_SUMMER
//    COLORMAP_SPRING
//    COLORMAP_COOL
//    COLORMAP_HSV
//    COLORMAP_PINK
//    COLORMAP_HOT
    return heap_map;

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2) {
    try {
        std::cout<<"get the img"<<std::endl;

        char* stereo_calibration_path="/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/ue_fov100_640_480.yml";
//        char* strero_engine_path="/home/hl/project/StereoAlgorithms-main/fast_acvnet_plus_generalization_opset16_480x640.onnx";
        char* strero_engine_path="/home/hl/project/StereoAlgorithms-main/PINTO_model_zoo/338_Fast-ACVNet/fast_acvnet_plus_onnx_gridsample/fast_acvnet_plus_generalization_opset16_384x960.onnx";
//        char* stereo_calibration_path="/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/ue_fov100_960_384.yml";
//        char* strero_engine_path="/home/hl/project/StereoAlgorithms-main/PINTO_model_zoo/338_Fast-ACVNet/fast_acvnet_plus_onnx_gridsample/fast_acvnet_plus_sceneflow_opset16_384x960.onnx";

        // 将 ROS 图像消息转换为 OpenCV 图像
        cv::Mat imageL = cv_bridge::toCvShare(msg1, "bgr8")->image;
        cv::Mat imageR = cv_bridge::toCvShare(msg2, "bgr8")->image;

        //init
        void * fastacvnet=Initialize(strero_engine_path,0,stereo_calibration_path);

        double t = (double)cv::getTickCount(); // 开始计时

        //x,y,z,r,g,b
        float*pointcloud=new float[imageL.cols*imageL.rows*6];
        cv::Mat disparity;
        for (size_t i = 0; i < 1;i++) //1000
        {
            cv::Mat imageL1=imageL.clone();
            cv::Mat imageR1=imageR.clone();
            //auto start = std::chrono::system_clock::now();

            //need RectifyImage
            RunFastACVNet_plus_RectifyImage(fastacvnet,imageL1,imageR1,pointcloud,disparity);

            //auto end = std::chrono::system_clock::now();
            //std::cout<<"time:"<<(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count())<<"ms"<<std::endl;
        }
        //disparity
//    cv::imwrite("/home/hl/project/StereoAlgorithms-main/disparity.jpg",disparity);

//    //heat map
        cv::Mat Heap_map=heatmap(disparity);
//    cv::imwrite("/home/hl/project/StereoAlgorithms-main/heatmap.jpg",Heap_map);

        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); // 计算经过的时间（秒）
        std::cout << "时间消耗 : " << t << " 秒." << std::endl;

//    //save pointcloud
//    std::fstream pointcloudtxt;
//    pointcloudtxt.open("/home/hl/project/StereoAlgorithms-main/pointcloud.txt",std::ios::out);
//    for (size_t i = 0; i < imageL.cols*imageL.rows*6; i+=6)
//    {
//        pointcloudtxt<<pointcloud[i]<<" "<<pointcloud[i+1]<<" "<<pointcloud[i+2]<<" "
//        <<pointcloud[i+3]<<" "<<pointcloud[i+4]<<" "<<pointcloud[i+5]<<std::endl;
//    }
//    pointcloudtxt.close();
        Release(fastacvnet);
        delete []pointcloud;
        pointcloud=nullptr;

        // 在这里处理图像
        // 例如：将两个图像转换为灰度图
//        cv::cvtColor(cv_image1, cv_image1, cv::COLOR_BGR2GRAY);
//        cv::cvtColor(cv_image2, cv_image2, cv::COLOR_BGR2GRAY);

        // 将处理后的图像转换回 ROS 图像消息
        //! 深度图转热度图
        sensor_msgs::ImagePtr processed_msg1 = cv_bridge::CvImage(msg1->header, "bgr8", Heap_map).toImageMsg();
//        sensor_msgs::ImagePtr processed_msg2 = cv_bridge::CvImage(msg2->header, "mono8", cv_image2).toImageMsg();

        image_publisher1.publish(processed_msg1);
//        image_publisher2.publish(processed_msg2);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("转换图像时出错: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastACV");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub1(nh, "/camera/image_raw1", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub2(nh, "/camera/image_raw2", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub1, image_sub2, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    image_publisher1 = nh.advertise<sensor_msgs::Image>("/processed_image1", 1);
//    image_publisher2 = nh.advertise<sensor_msgs::Image>("/processed_image2", 1);

    ros::spin();//     ros::spin();: 这会使您的程序进入一个无限循环，不断地调用可用的回调函数。它适用于那些不需要执行额外循环任务的节点，只是简单地等待并响应消息。    ros::spinOnce();: 这会处理所有当前的回调函数，但只执行一次，不会进入无限循环。这通常用在需要在主循环中执行其他任务的情况下，例如在主循环中发布消息、进行定时计算等。    由于您使用了 ros::spin();，主函数不会执行任何额外的代码，直到节点被关闭。因此，如果您只是接收消息并在回调函数中处理这些消息，而不需要执行其他定时任务，那么在主函数中使用 ros::Rate 是不必要的。
    return 0;

    char* stereo_calibration_path="/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/test/StereoCalibration.yml";
    char* strero_engine_path="/home/hl/project/StereoAlgorithms-main/fast_acvnet_plus_generalization_opset16_480x640.onnx";
    
    cv::Mat imageL=cv::imread("/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/test/left0.jpg");
    cv::Mat imageR=cv::imread("/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/test/right0.jpg");
    
    //init
    void * fastacvnet=Initialize(strero_engine_path,0,stereo_calibration_path);

    double t = (double)cv::getTickCount(); // 开始计时

    //x,y,z,r,g,b
    float*pointcloud=new float[imageL.cols*imageL.rows*6];
    cv::Mat disparity;
    for (size_t i = 0; i < 1;i++) //1000
    {
        cv::Mat imageL1=imageL.clone();
        cv::Mat imageR1=imageR.clone();
        //auto start = std::chrono::system_clock::now();
        
        //need RectifyImage
        RunFastACVNet_plus_RectifyImage(fastacvnet,imageL1,imageR1,pointcloud,disparity);
        
        //auto end = std::chrono::system_clock::now();
		//std::cout<<"time:"<<(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count())<<"ms"<<std::endl;
    }
    //disparity
//    cv::imwrite("/home/hl/project/StereoAlgorithms-main/disparity.jpg",disparity);

//    //heat map
    cv::Mat Heap_map=heatmap(disparity);
//    cv::imwrite("/home/hl/project/StereoAlgorithms-main/heatmap.jpg",Heap_map);

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency(); // 计算经过的时间（秒）
    std::cout << "时间消耗 : " << t << " 秒." << std::endl;

    cv::imshow("Heap_map",Heap_map);
    cv::waitKey(0);

//    //save pointcloud
//    std::fstream pointcloudtxt;
//    pointcloudtxt.open("/home/hl/project/StereoAlgorithms-main/pointcloud.txt",std::ios::out);
//    for (size_t i = 0; i < imageL.cols*imageL.rows*6; i+=6)
//    {
//        pointcloudtxt<<pointcloud[i]<<" "<<pointcloud[i+1]<<" "<<pointcloud[i+2]<<" "
//        <<pointcloud[i+3]<<" "<<pointcloud[i+4]<<" "<<pointcloud[i+5]<<std::endl;
//    }
//    pointcloudtxt.close();
    Release(fastacvnet);
    delete []pointcloud;
    pointcloud=nullptr;
    return 0;
}