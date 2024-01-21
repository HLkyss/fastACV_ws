# fastACV_ws
测试FastACVNet_plus的ros节点
尝试了一下在自己数据集上的效果，试了几个模型，感觉效果没那么稳定。
ROS工作空间：fastACV_ws
来源：
源码：https://github.com/gangweiX/Fast-ACVNet
源码(tensorrt加速版)：https://github.com/pcb9382/StereoAlgorithms/tree/main
模型下载：https://github.com/PINTO0309/PINTO_model_zoo

运行main_min.cpp和img_pub_960_384.cpp（将原数据集文件夹960*540的图片转换成960*384发布）测试，效果：
<img src="https://github.com/HLkyss/fastACV_ws/assets/69629475/f6278b42-004d-4e94-acd8-6ace493ad2b7" width="600"> <br />
<img src="https://github.com/HLkyss/fastACV_ws/assets/69629475/e8ac224b-ba66-4b17-a6c6-a53e2c6c889f" width="600"> <br />
<img src="https://github.com/HLkyss/fastACV_ws/assets/69629475/d3d168c0-8d4b-48c7-8784-42f8ff163714" width="600"> <br />
<img src="https://github.com/HLkyss/fastACV_ws/assets/69629475/7dafc63c-864e-4d2e-bc04-7a603fe0d9b7" width="600"> <br />

