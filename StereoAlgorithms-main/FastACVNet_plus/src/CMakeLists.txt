cmake_minimum_required (VERSION 3.8)

project(fastacvnet_plus)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O0 -g -rdynamic -g2 -ggdb")

set(TensorRT_INCLUDE "/opt/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/include")
set(TensorRT_LIB "/opt/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib" )

SET(LIBRARY_OUTPUT_PATH "/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/lib")

SET(OpenCV_DIR "/usr/local/opencv320")
SET(OpenCV_INCLUDE_DIRS "/usr/local/opencv320/include")
SET(OpenCV_LIBRARIES "/usr/local/opencv320/lib")
FIND_PACKAGE(OpenCV 3.2)

find_package(CUDA 11.2 REQUIRED)


include_directories("/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/include")
include_directories("/home/hl/project/StereoAlgorithms-main/common")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/local/cuda-11.2/include")

file(GLOB lib_header "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB common_header "/home/hl/project/StereoAlgorithms-main/common/*.h")

file(GLOB lib_src "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB common_src "/home/hl/project/StereoAlgorithms-main/common/*.cpp")

include_directories(${TensorRT_INCLUDE})
link_directories(${TensorRT_LIB})

cuda_add_library(${PROJECT_NAME} SHARED ${lib_src} ${common_src} FastACVNet_plus_preprocess.cu)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_libraries(${PROJECT_NAME} "/usr/local/cuda-11.2/targets/aarch64-linux/lib/libcudart.so")
else()
    target_link_libraries(${PROJECT_NAME} "/usr/local/cuda-11.2/lib64/libcudart.so")
endif()
target_link_libraries(${PROJECT_NAME} "${TensorRT_LIB}/libnvinfer.so")
target_link_libraries(${PROJECT_NAME} "${TensorRT_LIB}/libnvonnxparser.so")
target_link_libraries(${PROJECT_NAME} "${TensorRT_LIB}/libnvinfer_plugin.so")
#target_link_libraries(${PROJECT_NAME} "${TensorRT_LIB}/libcudnn.so")
target_link_libraries(${PROJECT_NAME} "/usr/local/cuda-11.2/targets/x86_64-linux/lib/libcudnn.so")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
