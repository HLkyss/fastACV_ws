# CMakeList.txt: NuctechPersonBagBinding 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)
project(fastacvnet_plus_demo)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -Wall -g")

set(TensorRT_INCLUDE "/opt/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/include")
set(TensorRT_LIB "/opt/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib")

#源文件
file(GLOB test_src "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

#头文件
file(GLOB test_header "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
#头文件搜索目录
SET(OpenCV_DIR "/usr/local/opencv320")
SET(OpenCV_INCLUDE_DIRS "/usr/local/opencv320/include")
SET(OpenCV_LIBRARIES "/usr/local/opencv320/lib")
FIND_PACKAGE(OpenCV 3.2)
include_directories("/home/hl/project/StereoAlgorithms-main")
include_directories("/home/hl/project/StereoAlgorithms-main/FastACVNet_plus/include")

# 将源代码添加到此项目的可执行文件。
add_executable(fastacvnet_plus_demo main.cpp ${test_header})
add_executable(fastacvnet_plus_demo2 main_min.cpp ${test_header})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} fastacvnet_plus)
target_link_libraries(fastacvnet_plus_demo2 ${OpenCV_LIBS})
target_link_libraries(fastacvnet_plus_demo2 fastacvnet_plus)
