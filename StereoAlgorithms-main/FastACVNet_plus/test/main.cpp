#include"FastACVNet_plus_Algorithm.h"
#include<iostream>

#include<chrono>


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
    cv::applyColorMap(abs_map,heap_map,cv::COLORMAP_JET);
    return heap_map;

}


int main()
{

//    char* stereo_calibration_path="/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/test/StereoCalibration.yml";
//    char* strero_engine_path="/home/hl/project/StereoAlgorithms-main/fast_acvnet_plus_generalization_opset16_480x640.onnx";
    char* stereo_calibration_path="/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/ue_fov100_960_384.yml";
//    char* strero_engine_path="/home/hl/project/StereoAlgorithms-main/PINTO_model_zoo/338_Fast-ACVNet/fast_acvnet_plus_onnx_gridsample/fast_acvnet_plus_sceneflow_opset16_384x960.onnx";
//    char* strero_engine_path="/home/hl/project/StereoAlgorithms-main/PINTO_model_zoo/338_Fast-ACVNet/fast_acvnet_plus_onnx_gridsample/fast_acvnet_plus_kitti_2015_opset16_384x960.onnx";
    char* strero_engine_path="/home/hl/project/StereoAlgorithms-main/PINTO_model_zoo/338_Fast-ACVNet/fast_acvnet_plus_onnx_gridsample/fast_acvnet_plus_generalization_opset16_384x960.onnx";

//    cv::Mat imageL=cv::imread("/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/test/cam0.jpg");
//    cv::Mat imageR=cv::imread("/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/test/cam1.jpg");
    cv::Mat imageL=cv::imread("/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/test/cam0.jpg");
    cv::Mat imageR=cv::imread("/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/test/cam1.jpg");
    
    //init
    void * fastacvnet=Initialize(strero_engine_path,0,stereo_calibration_path);

    //x,y,z,r,g,b
    float*pointcloud=new float[imageL.cols*imageL.rows*6];
    cv::Mat disparity;
    for (size_t i = 0; i < 1000;i++)
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
    cv::imwrite("/home/hl/project/StereoAlgorithms-main/disparity.jpg",disparity);

    //heat map
    cv::Mat Heap_map=heatmap(disparity);
    cv::imwrite("/home/hl/project/StereoAlgorithms-main/heatmap.jpg",Heap_map);

    //save pointcloud
    std::fstream pointcloudtxt;
    pointcloudtxt.open("/home/hl/project/StereoAlgorithms-main/pointcloud.txt",std::ios::out);
    for (size_t i = 0; i < imageL.cols*imageL.rows*6; i+=6)
    {
        pointcloudtxt<<pointcloud[i]<<" "<<pointcloud[i+1]<<" "<<pointcloud[i+2]<<" "
        <<pointcloud[i+3]<<" "<<pointcloud[i+4]<<" "<<pointcloud[i+5]<<std::endl;
    }
    pointcloudtxt.close();
    Release(fastacvnet);
    delete []pointcloud;
    pointcloud=nullptr;
    return 0;
}