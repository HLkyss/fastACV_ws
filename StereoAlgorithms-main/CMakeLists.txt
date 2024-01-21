cmake_minimum_required (VERSION 3.8)
project (StereoAlgorithmDemo)


# tensorrt path
set(TensorRT_INCLUDE "/opt/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/include" CACHE INTERNAL "TensorRT Library include location")
set(TensorRT_LIB "/opt/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib" CACHE INTERNAL "TensorRT Library include location")

#添加子目
#add_subdirectory("RAFTStereo")
#add_subdirectory("HitNet")
#add_subdirectory("CREStereo")
add_subdirectory("FastACVNet_plus")  #如果编译请使用TensorRT-8.6+
#add_subdirectory("Stereo_Calibration")

