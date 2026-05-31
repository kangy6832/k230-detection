# Micro PnP Plan
运行在正点原子 k230d box 上的黑色正方形框角点空间坐标检测程序。
## 1. 目标
- 实现一个能够在正点原子 k230d box 上运行的黑色正方形框角点空间坐标检测程序。
- 该程序能够准确地检测黑色正方形框的角点，并计算出它们在空间中的坐标。
## 2. 正方形框属性
- 颜色：黑色
- 形状：正方形
- 角点数量：4
- 边长：10cm
- 边框宽度：0.3cm
## 3. 检测方案
- 参考 /深入研究.md 和 /正点原子K230D BOX黑色正方形目标三维空间坐标检测算法实现方案.md 中的方案。
## 4. 参考网址
一、官方技术文档（核心权威来源）
1. 正点原子官方文档
    [K230D BOX 在线文档首页](https://wiki.alientek.com/docs/category/k230d-box/)
    [K230D BOX 产品介绍](https://wiki.alientek.com/docs/Boards/Kendryte/DNK230D/start-guide/k230d-box-introduction/)
    [K230D BOX 固件烧录教程](https://wiki.alientek.com/docs/Boards/Kendryte/DNK230D/start-guide/firmware-flash/)
    [正点原子 K230D BOX 产品详情页](https://www.alientek.com/Product_Details/130.html)

2. 嘉楠科技 CanMV 官方文档
    [CanMV K230 Sensor 模块 API 手册](https://developer.canaan-creative.com/k230_canmv/en/main/api/mpp/K230_CanMV_Sensor_Module_API_Manual.html)
    [CanMV K230 Image 模块 API 手册](https://www.kendryte.com/k230_canmv/en/main/api/openmv/image.html)
    [CanMV K230 nncase_runtime 模块 API 手册](https://developer.canaan-creative.com/k230_canmv/zh/main/zh/api/nncase/K230_CanMV_nncase_runtime_API%E6%89%8B%E5%86%8C.html)
    [CanMV K230 官方教程](https://www.kendryte.com/k230/en/v1.7/CanMV_K230_Tutorial.html)
    [CanMV K230 AI Demo 文档](https://www.kendryte.com/k230_canmv/en/v1.6/example/ai/AI_Demo_User_Manual.html)

二、技术社区与开发者分享
    [CanMV cv_lite.rgb888_pnp_distance_from_corners 函数使用教程](https://wenku.csdn.net/answer/1bjz9tgyvp2m)
    [K230 cv_lite 模块介绍与性能对比](https://www.yahboom.net/public/upload/upload-html/1755674597/1.Introduction%20to%20cv_lite.html)
    [OpenCV solvePnP 函数参数详解与使用示例](https://blog.csdn.net/jndingxin/article/details/145113294)
    [PnP 位姿估计算法原理与不同方法对比](https://blog.csdn.net/qq_30547073/article/details/78656795)
    [单目视觉 PnP 位姿估计实战教程](https://woteq.com/step-by-step-guide-to-solvepnp-in-opencv-with-python-examples/)

三、计算机视觉理论基础
    [OpenCV 官方文档：透视 - n - 点 (PnP) 姿态计算](https://docs.opencv.ac.cn/4.10.0/d5/d1f/calib3d_solvePnP.html)
    [OpenCV 官方文档：相机标定与 3D 重建](https://www.opencv.org.cn/opencvdoc/2.3.2/html/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)