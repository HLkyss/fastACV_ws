cmake_minimum_required (VERSION 3.8)
project (fastACV)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O0 -g -rdynamic -g2 -ggdb")

# tensorrt path
set(TensorRT_INCLUDE "/opt/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/include" CACHE INTERNAL "TensorRT Library include location")
set(TensorRT_LIB "/opt/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib" CACHE INTERNAL "TensorRT Library include location")

SET(LIBRARY_OUTPUT_PATH "/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/lib")

set(cv_bridge_DIR /home/hl/Downloads/vision_opencv-noetic/cv_bridge/install/install320/usr/local/share/cv_bridge/cmake)  #在find_package前面
find_package(cv_bridge)
SET(OpenCV_DIR "/usr/local/opencv320")
SET(OpenCV_INCLUDE_DIRS "/usr/local/opencv320/include")
SET(OpenCV_LIBRARIES "/usr/local/opencv320/lib")
FIND_PACKAGE(OpenCV 3.2)

set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.2/)
find_package(CUDA 11.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        )

include_directories("/home/hl/project/StereoAlgorithms-main")
include_directories("/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/include")
include_directories("/home/hl/project/StereoAlgorithms-main/common")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/local/cuda-11.2/include")
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
)

catkin_package()

#file(GLOB lib_header "/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/*.h")
file(GLOB common_header "/home/hl/project/StereoAlgorithms-main/common/*.h")

file(GLOB lib_src "/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/FastACVNet_plus_Algorithm.cpp" "/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/TRTFastACVNet_plus.cpp")
file(GLOB common_src "/home/hl/project/StereoAlgorithms-main/common/*.cpp")

#源文件
file(GLOB test_src "/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/main_min.cpp")
#头文件
#file(GLOB test_header "/home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/*.h")

include_directories(${TensorRT_INCLUDE})
link_directories(${TensorRT_LIB})

cuda_add_library(fastacvnet_plus SHARED ${lib_src} ${common_src} /home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/FastACVNet_plus_preprocess.cu)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_libraries(fastacvnet_plus "/usr/local/cuda-11.2/targets/aarch64-linux/lib/libcudart.so")
else()
    target_link_libraries(fastacvnet_plus "/usr/local/cuda-11.2/lib64/libcudart.so")
endif()
target_link_libraries(fastacvnet_plus "${TensorRT_LIB}/libnvinfer.so")
target_link_libraries(fastacvnet_plus "${TensorRT_LIB}/libnvonnxparser.so")
target_link_libraries(fastacvnet_plus "${TensorRT_LIB}/libnvinfer_plugin.so")
#target_link_libraries(${PROJECT_NAME} "${TensorRT_LIB}/libcudnn.so")
target_link_libraries(fastacvnet_plus "/usr/local/cuda-11.2/targets/x86_64-linux/lib/libcudnn.so")
target_link_libraries(fastacvnet_plus ${OpenCV_LIBS})

add_executable(fastacvnet_plus_demo2 /home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/main_min.cpp ${test_header})
target_link_libraries(fastacvnet_plus_demo2 ${OpenCV_LIBS} ${catkin_LIBRARIES} ${cv_bridge_LIBRARIES})
target_link_libraries(fastacvnet_plus_demo2 fastacvnet_plus)

add_executable(img_pub_test /home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/img_pub_test.cpp ${test_header})
target_link_libraries(img_pub_test ${OpenCV_LIBS} ${catkin_LIBRARIES} ${cv_bridge_LIBRARIES})
add_executable(img_pub_640_480 /home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/img_pub_640_480.cpp ${test_header})
target_link_libraries(img_pub_640_480 ${OpenCV_LIBS} ${catkin_LIBRARIES} ${cv_bridge_LIBRARIES})
add_executable(img_pub_960_384 /home/hl/project/StereoAlgorithms-main/fastACV_ws/src/fastACV/src/img_pub_960_384.cpp ${test_header})
target_link_libraries(img_pub_960_384 ${OpenCV_LIBS} ${catkin_LIBRARIES} ${cv_bridge_LIBRARIES})
