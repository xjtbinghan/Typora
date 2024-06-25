# 1.源码解析

## 单目模式tum数据集的入口文件：mono_tum.cc

该代码由两个函数组成，一个main函数，一个是读取图像的Loadimages函数。

**main函数的核心操作为：**

1.读取文件，获得每一张图像的路径

![image-20230807160944286](/home/hanbing/.config/Typora/typora-user-images/image-20230807160944286.png)

2.**初始化ORB_SLAM2 System的对象SLAM**

![image-20230807144004299](/home/hanbing/.config/Typora/typora-user-images/image-20230807144004299.png)

2.使用for循环，将每一帧和时间戳数据送入**SLAM.TrackMonocular**

3.使用第二个for循环，统计处理所有图像使用的时间。再保存相机的轨迹到一个txt文件中。SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

![image-20230807115842065](/home/hanbing/.config/Typora/typora-user-images/image-20230807115842065.png)

![image-20230807115904231](/home/hanbing/.config/Typora/typora-user-images/image-20230807115904231.png)

![image-20230807120146589](/home/hanbing/.config/Typora/typora-user-images/image-20230807120146589.png)

**Loadimages函数的核心操作是**：

使用ifstream 创建对象f ，用f打开file，获得每一个图像的文件名赋给vstrImageFilenames。这个代码打开的是tum数据集。

![image-20230807143007750](/home/hanbing/.config/Typora/typora-user-images/image-20230807143007750.png)