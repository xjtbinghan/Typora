[TOC]

# Orbslam2 System总图

![image-20230829212657419](/home/hanbing/.config/Typora/typora-user-images/image-20230829212657419.png)

![image-20231002160936350](/home/hanbing/.config/Typora/typora-user-images/image-20231002160936350.png)



# System类之成员函数

System.cc是系统主类，通过Examples里面的Monocular、Stereo或rgbd.cc的入口文件，先调用System类**创建对应的system对象**，然后根据传感器类型，调用相应的入口函数TrackMonocular、TrackRGBD或TrackStereo，将数据传入SLAM系统开启SLAM流程。system文件包括以下几部分：

**1.System类的构造函数**，包括初始化各成员变量、创建3大线程和建立线程间的通信；

**2.根据传感器的类型的跟踪函数**，包括TrackStereo，TrackRGBD，TrackMonocular；

**3.System类的其他函数**。



## 1.System：系统的构造函数,包括初始化各成员变量、创建3大线程和建立线程间的通信

### step 0 初始化参数

0.1 给构造函数的参数和成员变量赋予初值

![image-20230815095831741](/home/hanbing/.config/Typora/typora-user-images/image-20230815095831741.png)

**输入参数：**

1. `strVocFile`：一个 `std::string` 类型的参数，表示 `ORB` 词汇表的文件路径。`ORB` 词汇表用于在图像中提取特征点并进行描述子匹配，是视觉 SLAM 算法的核心组件之一。

2. `strSettingsFile`：一个 `std::string` 类型的参数，表示配置文件的路径。这个配置文件包含了系统运行所需的各种参数和设置，如相机内参、校正系数、ORB 特征的相关参数等。

3. `sensor`：一个枚举类型 `eSensor` 参数，表示传感器类型。这个参数决定了系统将如何处理输入的图像数据，可以是 `MONOCULAR`（单目相机）、`STEREO`（立体相机）或 `RGBD`（RGB-D 相机）。

4. `bUseViewer`：一个布尔类型的参数，表示是否使用查看器（`Viewer`）。如果设置为 `true`，系统将创建一个可视化界面来实时显示图像、关键帧和地图点等信息。如果设置为 `false`，系统将不会显示图像界面。

   这些参数在构造函数内部被用于初始化 `System` 类的成员变量，从而配置和启动整个 `ORB-SLAM2` 系统。通过这些参数的不同组合，可以适应不同类型的传感器和应用场景。

**成员变量：**

1. `mpViewer`：这是一个指向 `Viewer` 类对象的指针，用于实时地显示图像、关键帧和地图点等信息。`Viewer` 是用于可视化 `ORB-SLAM2` 系统运行过程的一个组件。通过这个指针，系统可以与查看器进行交互，例如在跟踪过程中将结果显示出来。

2. `mbReset`：这是一个布尔类型的标志， 用于表示是否需要对系统进行重置。当该标志为 `true` 时，系统会进行重置操作，以清空已有的状态和数据，重新开始运行。

3. `mbActivateLocalizationMode`：这也是一个布尔类型的标志，用于表示是否需要激活定位模式。当该标志为 `true` 时，系统会进入定位模式，这可能会涉及到停止某些线程或者切换系统的运行状态。

4. `mbDeactivateLocalizationMode`：这同样是一个布尔类型的标志，用于表示是否需要停用定位模式。当该标志为 `true` 时，系统会退出定位模式，这可能涉及到重新启动之前停止的线程或者切换系统状态。

5. `mSensor`：这是一个eSensor数据类型的枚举向量，用于表示传感器的类别。

   这些成员变量在 `System` 类的构造函数中被初始化，并在不同的地方用于控制系统的行为。通过设置这些标志，可以在系统运行过程中对系统的状态和行为进行动态的控制和调整。这种机制使得 `ORB-SLAM2` 能够根据需要在不同模式下运行，进行重置或切换功能，以满足特定的应用需求。

### step 1  初始化各成员变量

1.1读取相机配置文件信息

![image-20230807161609621](/home/hanbing/.config/Typora/typora-user-images/image-20230807161609621.png)

1.2创建ORB词袋

![image-20230807164511704](/home/hanbing/.config/Typora/typora-user-images/image-20230807164511704.png)

1.3初始化关键帧数据库，主要保存ORB描述子倒排索引（即根据描述子查找拥有该描述子的关键帧）

![image-20230807165151088](/home/hanbing/.config/Typora/typora-user-images/image-20230807165151088.png)

1.4初始化地图

![image-20230807165223439](/home/hanbing/.config/Typora/typora-user-images/image-20230807165223439.png)

1.5初始化FrameDrawer和MapDrawer

![image-20230807165252975](/home/hanbing/.config/Typora/typora-user-images/image-20230807165252975.png)

### step 2 创建3大线程（Tracking、Localmapping、LoopClosing）

**2.1-2.3 初始化并开启3大线程**

基于前面初始化的各成员变量，将各个成员变量作为参数开启线程。

为什么Tracking线程在system类中没有对应的std::thread ，而Localmapping、LoopClosing线程有对应的std::thread呢？因为从代码上看，tracking线程就是主线程，Localmapping、LoopClosing是其子线程，tracking通过持有它们的指针而控制子线程。（ps：虽然在变成上构成父子关系，但是逻辑上我们认为这三者是并行的）

![image-20230814160157571](/home/hanbing/.config/Typora/typora-user-images/image-20230814160157571.png)

**2.4初始化查看器并开启线程**

![image-20230814160242411](/home/hanbing/.config/Typora/typora-user-images/image-20230814160242411.png)

### step 3 建立各个线程之间的通信

![image-20230814160017835](/home/hanbing/.config/Typora/typora-user-images/image-20230814160017835.png)



## 2.根据传感器的类型的跟踪函数，TrackStereo，TrackRGBD，TrackMonocular；

system对象的主线程就是跟踪线程，针对不同的传感器有用于三个的跟踪函数，内部实现调用mpTracker的主函数GrabImageMonocular（GrabImageRGBD、GrabImageStereo）方法

| 传感器类型 | 用于跟踪的成员函数                                           |
| ---------- | ------------------------------------------------------------ |
| Stereo     | cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp) |
| RGBD       | cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp) |
| Monocular  | cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp) |

**以TrackRGBD为例，TrackRGBD共有5个步骤，分别为：**

**step 1：**根据系统成员变量mSensor检查传感器类型；

**step 2：**如果mbActivateLocalizationMode变量设置为True，停止本地地图构建线程，并将mpTracker切换到仅跟踪模式，
               如果mbDeactivateLocalizationMode 被设置为 True，则取消仅跟踪模式，释放本地地图构建线程；（初始化时均为flase）

**step 3：**重置检查，如果标志 mbReset 被设置为 true，则调用跟踪器的 Reset 方法进行系统重置；（初始化时为flase）

**step 4 (核心)：**调用tracking类（跟踪器）的 GrabImageRGBD 方法进行跟踪，传入 RGB 图像、深度图像和时间戳，返回估计的相机姿态；

**step 5：**更新跟踪状态和关键点信息，将跟踪器的状态mState、被跟踪的地图点和未校准的关键点保存到系统变量中。

```c++
cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    //step 1 检查传感器类型和错误处理 
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    
    //step 2 模式检查和模式切换（本地地图构建和仅跟踪模式切换）
    {
        //使用互斥锁（mMutexMode）锁定模式检查和切换操作，以确保多线程环境下的安全。
        unique_lock<mutex> lock(mMutexMode);

        //如果标志 mbActivateLocalizationMode 被设置为 true，则请求本地地图构建线程停止，并等待其实际停止。System类初始化时为false
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }
            //一旦本地地图构建线程停止，将跟踪器（mpTracker）切换到仅跟踪模式（调用 InformOnlyTracking(true)）。
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }

        //如果标志 mbDeactivateLocalizationMode 被设置为 true，则取消仅跟踪模式，释放本地地图构建线程。System类初始化时为false
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }
    //step 3 重置检查
    {
    //使用互斥锁（mMutexReset）锁定重置检查和重置操作，以确保多线程环境下的安全。
    unique_lock<mutex> lock(mMutexReset);
    //如果标志 mbReset 被设置为 true，则调用跟踪器的 Reset 方法进行系统重置。System类初始化时为false
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }
    //step 4 调用跟踪器进行跟踪
    //调用跟踪器的 GrabImageRGBD 方法，传入 RGB 图像、深度图像和时间戳，进行相机姿态估计。
    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    //step 5 更新跟踪状态和关键点信息
    //使用互斥锁（mMutexState）锁定状态更新操作，以确保多线程环境下的安全。
    unique_lock<mutex> lock2(mMutexState);
    //将跟踪器的状态（mpTracker->mState）、被跟踪的地图点（mpTracker->mCurrentFrame.mvpMapPoints）
    //和未校准的关键点（mpTracker->mCurrentFrame.mvKeysUn）保存到系统的成员变量中。
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    //将相机姿态（Tcw）返回。
    return Tcw;
}
```

