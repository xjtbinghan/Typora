[TOC]

# Tracking总述

​		Tracking线程是system中创建3大线程时进行初始化的，以Tracking::GrabImageRGBD作为入口和框架，以track()为核心，对每一帧追踪位姿。tracking线程也是局部建图和回环检测的前端，通过插入关键帧引出之后的流程。

​		Tracking主要包括三个阶段，第一个阶段包括三种跟踪方法：用参考关键帧来跟踪、恒速模型跟踪、重定位跟踪，它们的目的是保证能够“跟的上”，但估计出来的位姿可能没那么准确。第二个阶段是局部地图跟踪，将当前帧的局部关键帧对应的局部地图点投影到该帧，得到更多的特征点匹配关系，对第一阶段的位姿再次优化得到相对准确的位姿。第三个阶段是决定是否新建关键帧，插入关键帧之后引出建图流程，生成新的地图点。建图流程中新的地图点又将反馈给下一轮跟踪。

### 流程图

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20231002160822097.png" alt="image-20231002160822097" style="zoom: 50%;" />

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20231002160837915.png" alt="image-20231002160837915" style="zoom: 45%;" />

# 一、构造函数——Tracking::Tracking

system中创建3大线程时初始化tracking，后面的参数是在初始化线程之前初始化的各个成员变量。

```c++
 mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);
```

## step 0 初始化Tracking的参数和成员变量

![image-20230815095703313](/home/hanbing/.config/Typora/typora-user-images/image-20230815095703313.png)

### 输入参数

​		所有的输入参数都是system初始化tracking时传入的，其中：strSettingPath和sensor（7-8）是由system类的输入参数决定的，即我们初始输入的配置文件和传感器类型。其他的参数（1-6）都是在system中初始化之后的各个类对象，分别表示各个不同的组件。由于tracking需要通信和控制其他组件，所以在输入参数中接受其他组件的类对象。

1. `System *pSys`：这是一个指向 `System` 类对象的指针，用于与整个系统进行通信和交互。在 `Tracking` 类中，可能需要调用系统的其他模块来进行定位、闭环检测等操作。
2. `ORBVocabulary* pVoc`：这是一个指向 `ORBVocabulary` 类对象的指针，表示用于提取和匹配特征点的词汇表。`ORBVocabulary` 是 `ORB-SLAM2` 中用于描述子匹配的关键组件之一。
3. `FrameDrawer *pFrameDrawer`：这是一个指向 `FrameDrawer` 类对象的指针，用于在系统中绘制当前帧的图像和特征点。在可视化阶段，可能会用到这个对象来绘制图像。
4. `MapDrawer *pMapDrawer`：这是一个指向 `MapDrawer` 类对象的指针，用于在系统中绘制地图。在可视化阶段，可能会用到这个对象来绘制地图点。
5. `Map *pMap`：这是一个指向 `Map` 类对象的指针，表示地图对象。`Map` 类用于存储地图点、关键帧等信息。
6. `KeyFrameDatabase* pKFDB`：这是一个指向 `KeyFrameDatabase` 类对象的指针，表示关键帧数据库。关键帧数据库用于存储和管理关键帧的信息。
7. `const string &strSettingPath`：这是一个常引用，表示配置文件的路径。配置文件包含了系统运行所需的各种参数和设置。
8. `const int sensor`：这是一个整数，表示传感器类型。可以是 `System::MONOCULAR`（单目相机）、`System::STEREO`（立体相机）或 `System::RGBD`（RGB-D 相机）。

### 成员变量

​		下表中，1.`mState`、3.`mbOnlyTracking`、4.`mbVO`、7.`mpInitializer`、9.`mpViewer`和13.`mnLastRelocFrameId` 是在tracking中初始化的，其他的变量使用输入参数赋予初始值，全部是来源于system。

1. `mState`：是定义的枚举类型变量，表示跟踪的阶段，初始化为`NO_IMAGES_YET`（尚未有图像输入）（详见Tracking线程之Tracking.h）；

2. `mSensor`：是int类型的成员变量，用于接收传入的传感器类型枚举变量（来自system）。它可以是 `System::MONOCULAR`（=0）、`System::STEREO`（=1）或 `System::RGBD`（=2）；
3. `mbOnlyTracking`：是定义的一个布尔类型的标志，表示是否仅进行跟踪而不执行定位等操作。初始化为false；
4. `mbVO`：是定义的一个布尔类型的标志，表示是否启用了视觉里程计（Visual Odometry）模式。当为 `true` 时，系统会使用视觉里程计估计相机运动，初始化为false；
5. `mpORBVocabulary`：一个指向 `ORBVocabulary` 类对象的指针，用于存储 `ORB` 词汇表，用于特征点匹配。接收输入参数ORBVocabulary* pVoc的赋值（来自system）；
6. `mpKeyFrameDB`：一个指向 `KeyFrameDatabase` 类对象的指针，表示关键帧数据库，用于存储和管理关键帧信息。接收输入参数KeyFrameDatabase* pKFDB的赋值（来自system）；
7. `mpInitializer`：一个指向 `Initializer` 类对象的指针，用于初始化系统。在初始化阶段，可能会用到这个对象来计算初始相机位姿。初始化为空指针；
8. `mpSystem`：一个指向 `System` 类对象的指针，用于与整个系统进行交互和通信。接收输入参数System *pSys的赋值（来自system）；
9. `mpViewer`：一个指向 `Viewer` 类对象的指针，用于实时地显示图像、关键帧和地图点等信息，初始化为空指针；
10. `mpFrameDrawer`：一个指向 `FrameDrawer` 类对象的指针，用于在系统中绘制当前帧的图像和特征点，接收输入参数FrameDrawer *pFrameDrawer的赋值（来自system）；
11. `mpMapDrawer`：一个指向 `MapDrawer` 类对象的指针，用于在系统中绘制地图，接收输入参数MapDrawer *pMapDrawer的赋值（来自system）；
12. `mpMap`：一个指向 `Map` 类对象的指针，表示地图对象，用于存储地图点、关键帧等信息，接收输入参数Map *pMap的赋值（来自system）；
13. `mnLastRelocFrameId`：是定义的一个int类型变量，表示上一次重定位操作时的关键帧的 ID。在重定位过程中，可能会用到这个变量来跟踪上一次成功重定位的帧。

## step 1 读取配置文件中的相机参数

```c++
Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // step 1 读取配置文件，将这些参数保存到类的成员变量中。
    // 1.1使用cv::FileStorage 创建fSettings对象读取YAML文件
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    // 1.2获取相机内参
    float fx = fSettings["Camera.fx"]; //x方向焦距
    float fy = fSettings["Camera.fy"]; //y方向焦距
    float cx = fSettings["Camera.cx"]; //x方向光心坐标
    float cy = fSettings["Camera.cy"]; //y方向光心坐标

    //1.3创建一个3x3的单位矩阵用来表示内参矩阵
    //      /fx  0  cx/
    //  K = /0  fy  cy/
    //      /0   0   1/
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);//将得到的内参矩阵 K 复制给类成员变量 mK，这个内参矩阵会在跟踪过程中用于相机的校正。

    //1.4 创建了4x1的矩阵 DistCoef，用于存储相机的畸变矫正系数,如果k3不为0 则变为5x1矩阵
    // [k1 k2 p1 p2 k3]
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    //1.5 将得到的畸变矫正系数矩阵复制给类成员变量mDistCoef
    DistCoef.copyTo(mDistCoef);//
    //1.6 双目基线值*焦距 baseline * fx（能观测到的最远距离）赋值给成员变量mbf
    mbf = fSettings["Camera.bf"]; 
    //1.7 帧率赋值给成员变量fps
    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    //1.8 最小帧数和最大帧数这些变量用于确定何时插入关键帧以及何时检查重定位。
    mMinFrames = 0;
    mMaxFrames = fps;
    //打印相机参数
    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    //1.9将图像RGB顺序赋值给成员变量mbRGB
    //图像的颜色顺序（0：BGR，1：RGB）
    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;
    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    //step 2 设置 ORB 特征提取器的参数
    .......

}
```

## step 2 读取配置文件中的ORB算法参数、构建ORB特征提取器对象

```c++
    //2.1 加载ORB算法参数储存在成员变量中
    int nFeatures = fSettings["ORBextractor.nFeatures"];       //特征点个数
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];//特征金字塔的尺度因子
    int nLevels = fSettings["ORBextractor.nLevels"];           //特征金字塔的层数
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];      //初始 FAST 特征提取阈值
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];      //若取不出足够的fast特征点则使用最小FAST 特征提取阈值
    
    //2.2 初始化 ORB 特征提取器对象
    //Tracking过程都会用到mpORBextractorLeft作为特征提取器
    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    //如果相机是双目(STEREO)，则也会为右目相机初始化一个 ORB 特征提取器
    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    //如果相机是单目(MONOCULAR)，在单目初始化的时候，使用mpIniORBextractor作为ORB特征提取器
    //初始化要求的特征点是平时的2倍
    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    //打印参数
    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
    
    //2.3 双面和RGBD的特殊参数

    //如果是双目和RGBD，则根据相机参数计算深度信息的阈值mThDepth（baseline * ThDepth）
    //用于判断3D点是远/近点，建图的时候会剔除远点
    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }
    //如果相机为 RGB-D 类型，从配置文件中读取 DepthMapFactor 参数，表示深度图的尺度因子，是相机本身的参数
    if(sensor==System::RGBD)
    {
        //如果mDepthMapFactor绝对值小于 0.00001（接近0）设置为 1 
        //否则取其倒数
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }
```



# 二、入口——cv::Mat Tracking::GrabImageRGBD

​		该函数算是真正意义上的Tracking流程的入口或是大框架，这个函数包括了构造帧，并把帧传入track()跟踪位姿，然后把跟踪到的位姿mTcw返回到系统中。

<img src="/home/hanbing/公共的/typora实验记录/ORB_SLAM2/assets/image-20231008095548989.png" alt="image-20231008095548989" style="zoom: 67%;" />

## step 0 暗线（调用关系）

​		以RGBD为例，GrabImageRGBD是system在跟踪环节中使用初始化之后的tracking对象调用的（在system的cv::Mat System::TrackRGBD方法中调用），入口参数为RGB图像+深度图+时间戳。

![image-20230815185209414](/home/hanbing/公共的/typora实验记录/ORB_SLAM2/assets/image-20230815185209414.png)

## step 1 接受RGB、BGR或RGBA、BGRA图像转化为灰度图mImGray

​		前者三通道、后者四通道，A可能表示透明度

## step 2 深度图缩放（RGBD特有）

​		由于RGBD相机自带深度图，这个深度图需要乘上个缩放因子，然后使用。单目和双目没有这个流程，只有step1 3 4 5。

## step 3 使用生成的灰度图构造frame对象

​		使用当前的数据构造Frame对象：使用生成的灰度图mImGray、缩放后的深度图imDepth、时间戳timestamp以及构造函数中初始化的各个成员变量mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth创建Frame对象。帧类的属性不仅包括图像，还包括特征点等一些系列的成员变量，在初始化Frame的时候，就已经对该帧的图像完成了ORB特征提取。**详见Frame.md**

![image-20231008094536596](/home/hanbing/公共的/typora实验记录/ORB_SLAM2/assets/image-20231008094536596.png)

## step 4 进入Track（）函数进行tracking流程

对构造的当前帧，调用 `Track` 函数来执行跟踪操作。

## step 5 输出世界坐标系到该帧相机坐标系的变换矩阵 Tcw

Track流程之后更新了成员变量，将Tcw进行返回，再反馈到system的cv::Mat System::TrackRGBD方法中。

![image-20231008094740413](/home/hanbing/公共的/typora实验记录/ORB_SLAM2/assets/image-20231008094740413.png)



# 三、主函数——void Tracking::Track()

## step 0 暗线（调用关系）

void Tracking::Track()是Tracking.cc的主函数，GrabImageRGBD（GrabImageStereo和GrabImageMonocular）都需要调用Track方法。

![img](https://img-blog.csdnimg.cn/20190605093523251.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L21veXUxMjM0NTY3ODk=,size_16,color_FFFFFF,t_70)

## Step 1 初始化

见（四、双目初始化Tracking::StereoInitialization()）（五、单目初始化Tracking::MonocularInitialization()）。初始化完成后会更新关键帧、地图点和地图数据库的成员变量，对之后帧的跟踪很关键。

## step 2 相机跟踪和定位

​		这部分是整个跟踪的核心，包括三大跟踪，跟踪局部地图和插入关键帧。		

### 三大跟踪		

​		三大跟踪的流程是：刚初始化完了和刚刚重定位完之后，使用参考关键帧跟踪，跟踪成功了就有了速度mVelocity，后面就使用恒速运动模型跟踪了；如果恒速运动模型跟踪失败，就进行参考关键帧跟踪，如果又失败，那么就采用重定位。重定位后，是使用参考关键帧跟踪，之后使用恒速运动模型...  

### 跟踪局部地图

​		参考关键帧跟踪、恒速模型跟踪、重定位跟踪，它们的目的是保证能够“跟的上”，但估计出来的位姿可能没那么准确。所以后面再使用局部地图的更多匹配信息来补充优化。局部地图跟踪将当前帧的局部关键帧对应的局部地图点投影到该帧，得到更多的特征点匹配关系，对第一阶段的位姿再次优化得到相对准确的位姿。

### 决定是否插入关键帧

​		整个地图的轨迹和路标点是依靠关键帧及其对应的地图点来表示的，而且参考关键帧跟踪、重定位和跟踪局部地图都需要使用关键帧和地图点的信息。初始地图的关键帧和地图点是在初始化流程中建立的。所以在每一轮跟踪的同时，应该逐渐更新关键帧和地图点。在均匀、可靠的原则上，逐个筛选并插入关键帧，使用该关键帧完成之后的局部建图和地图更新，并反馈给下一轮Track。

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230922184339693.png" alt="image-20230922184339693" style="zoom: 50%;" />



## step 3 存储帧位姿信息以便随后检索完整的相机轨迹

​		对于跟踪成功的帧，记录当前帧与参考关键帧的相对位姿Tcr，并把Tcr和参考帧对象储存在mlRelativeFramePoses和mlpReferences中，如此一来这两个成员变量可以计算每一帧的位姿。除此之外，使用mlFrameTimes记录每帧的时间戳，并返回当前这一次跟踪是否失败了，和LOST对比，成功返回false，失败就返回ture。

​		对于跟踪失败的帧，继续使用之前的Tcr的和参考帧，表示没有移动。mlblost设置为Ture。

![image-20230922173118074](/home/hanbing/.config/Typora/typora-user-images/image-20230922173118074.png)

![image-20231008111022300](/home/hanbing/公共的/typora实验记录/ORB_SLAM2/assets/image-20231008111022300.png)

# 四、初始化之双目/RGBD：Tracking::StereoInitialization()

​		恒速运动模型跟踪时，update那个函数操作不同

​		单目初始化没有用到局部建图，把建图、建立观测索引、更新共视图等全部放在主线程中。而双目把用到了局部建图，把一部分的工作（）放在了局部建图流程中完成。

# 五、初始化之单目：Tracking::MonocularInitialization()

## 单目初始化逻辑图

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230831161652079.png" alt="image-20230831161652079" style="zoom:67%;" />

step 2 

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230904203516131.png" alt="image-20230904203516131" style="zoom: 67%;" />

step 3

<img src="file:////home/hanbing/.config/QQ/nt_qq_8fd520cf964b05435dd14625633ac868/nt_data/Pic/2023-09/Ori/2bf4a51a0eb03d8741ca18403c02c8f8.jpeg" alt="img" style="zoom: 25%;" />

## step 0 暗线（调用关系）

​		主函数track()中被调用，完整调用线如下：Tracking::GrabImageRGBD—>Tracking::Track()—>Tracking::MonocularInitialization()。

## step 1 构建初始化器

​		单目初始化的要求是找到FeaturePoint>100的相邻帧，且这两帧的特征匹配数量nmatches>100。

​		在找到第一个满足条件（特征点>100）的帧之后使用该帧创建初始化器对象，1.0代表sigma，200代表iterations。详见Initializer.md中的构造函数。如果下一帧特征点<100，那么删除之前创建的初始化器再重头找。如果初始化器的下一帧特征点也>100,那么使用这两个帧暂且作为初始化的两帧，进行特征匹配。如果特征匹配得到的匹配数nmatches<100，那就也删除初始化器再重头找。

```c++
mpInitializer =  new Initializer(mCurrentFrame,1.0,200);
```

## step 2 计算初始帧与当前帧的匹配

​		首先构造ORBmatcher的对象，0.9为特征匹配筛选最优匹配时的最优和次优比例设定，最优匹配需要满足这个比例条件才算成功；true表示mbCheckOrientation为真，也就是在所有特征点匹配结束后还要再使用特征点主方向差直方图剔除非主流方向的匹配点。

​		其次使用SearchForInitialization()方法，找到了两帧之间的匹配，并将匹配的数目存入nmatches中。初始化时的特征匹配方法比较特殊，和跟踪环节都不一样。既没有初始位姿、地图点也没有使用DBoW词袋。而是直接将F1中的特征点坐标放入F2中，找以该坐标为中心的Window大小范围内的特征点作为候选匹配点，然后在候选匹配点中找最优匹配，再使用特征主方向差直方图剔除非主流匹配。详见ORBmatcher.md及其matcher.SearchForInitialization()方法。

```c++
 //初始化ORBmatcher对象，用于匹配
	ORBmatcher matcher(0.9,true);
	int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);
//mInitialFrame：初始帧对象
//mCurrentFrame：当前帧对象
//mvbPrevMatched：长度==F1中的特征点个数，索引代表F1的特征点索引，值代表与该F1特征点成功匹配的F2特征点的坐标pt。 std::vector<cv::Point2f> mvbPrevMatched;
//mvIniMatches：长度==F1中的特征点个数，索引代表F1的特征点索引，值代表与该F1特征点成功匹配的F2特征点的索引。
//100：GetFeaturesInArea搜索时的windowSize
```

## step 3 根据2D匹配求解R T 、3D点

​		这一部分涉及到整个单目初始化的理论，也就是如果通过两张图像的2D-2D匹配计算外参RT并三角化得到地图点。大概可以分为以下流程：

1. **根据2D匹配计算最优基础矩阵F和单应矩阵H：**RANSAC、特征点归一化、SVD求解最小二乘问题、逆归一化、卡方检验区分内外点；
2. **从F中恢复E，并从E中恢复4个RT：**本质矩阵和基本矩阵的关系，根据E的矩阵性质，使用SVD分解求解RT；
3. **根据RT三角化，区分内外点，并找出4个RT中正确的那个：**SVD分解 三角化。

​	详见Initializer::Initialize()和在视觉SLAM14讲中夹着的笔记，上面写了理论的推导。

```c++
//step 3 对初始帧和当前帧的匹配关系进行初始化位姿求解
        // 定义变量存储初始化后的旋转、平移和三维点信息
        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // 三角化的匹配关系（mvIniMatches）

        //使用Initialize对初始帧和当前帧的匹配关系进行初始化位姿求解
        //std::vector<cv::Point3f> mvIniP3D; 表示3D点位置
        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }
            //设置初始帧和当前帧的位姿
            //将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);
```

## step 4 创建初始地图，初始化全局BA

​		使用初始化的第一帧的光心作为世界坐标系原点，第一帧的位姿为I。创建初始化成功的两帧为关键帧、在Map中插入关键帧和地图点，初始化地图，更新关键帧共视关系（详见KeyFrame::UpdateConnections），全局BA优化关键帧位姿和地图点，前两个关键帧插入局部地图并注册为局部关键帧，把初始化的第二帧设置为了tracking流程的上一帧、上一个关键帧和参考关键帧。

​		这里面需要注意的是，关键帧是在Initialize()完成之后，才把初始帧mInitialFrame和当前帧mCurrentFrame设为关键帧。设为关键帧的目的是建立共视图和BA优化，因为这些使用关键帧完成的而不是普通帧。详见（六、创建初始地图—Tracking::CreateInitialMapMonocular()）

### 关键帧和地图点更新时刻

​		初始化成功后建立的初始地图会更新关键帧和地图点，这时会把初始化的两帧注册为关键帧，把三角化之后的内点注册为地图点，还会把初始化的第二帧暂时在作为参考关键帧，用于初始化之后第一次参考关键帧跟踪的参考帧。 

### 涉及的关键变量（Track、KF、MP）

| Track成员变量      | 类型                       | 意义                     | 描述                                                         |
| ------------------ | -------------------------- | ------------------------ | ------------------------------------------------------------ |
| mpInitializer      | Initializer*               | 初始化类指针             | 初始化之前设为NULL ，找到第一帧满足要求的帧时创建实例，之后如果没有找到两个满足条件的帧，再次设为NULL |
| mvbPrevMatched     | std::vector< cv::Point2f > | 储存F2特征点的坐标pt     | 表示初始化两帧的匹配关系，向量按照F1的特征点索引，值代表与该F1特征点成功匹配的F2特征点的坐标pt。 |
| mvIniMatches       | std::vector< int>          | 储存F2特征点的索引       | 表示初始化两帧的匹配关系，向量按照F1的特征点索引，值代表与该F1特征点成功匹配的F2特征点的索引。 |
| mvIniP3D           | std::vector<cv::Point3f >  | 储存三角化之后地图点坐标 | 向量按照按照F1的特征点索引，存储成功匹配和三角化的地图点的坐标。如果没有就是空。 |
| mInitialFrame      | Frame                      | 初始帧对象               | 初始化时的第一帧记为初始帧，该帧位姿为I，光心表示世界坐标系的中心。 |
| **KF和地图点变量** | **类型**                   | **意义**                 | **描述**                                                     |
| pKFini             | KeyFrame*                  | 初始关键帧1              | 初始化成功后把第一帧设为关键帧对象                           |
| pKFcur             | KeyFrame*                  | 初始关键帧2              | 初始化成功后把第二帧设为关键帧对象                           |
| pMP                | MapPoint*                  | 用于插入的临时地图点对象 | 初始化时对每个三角化出来的点都构造一个临时pMP对象指针，该对象携带三维坐标并绑定观测索引，插入到关键帧和地图中 |
| mvpLocalKeyFrames  | std::vector<KeyFrame*>     | 局部关键帧               | pKFini和pKFcur作为局部关键帧，跟踪局部地图环节进行更新。     |
| mvpLocalMapPoints  | std::vector<MapPoint*>     | 局部地图点               | 初始化得到的地图点作为局部地图点，跟踪局部地图时进行更新     |
| mpLocalMapper      | LocalMapping*              | 局部地图                 | 初始化之后更新了局部地图，之后更新局部地图在跟踪局部地图里。 |
|                    |                            |                          |                                                              |

# 五、单目初始化step 4创建初始地图—Tracking::CreateInitialMapMonocular()

## step 1 把构成初始化的初始帧和当前帧设置为关键帧

## step 2 把这俩关键帧插入地图中

## step 3 对初始化的地图点操作

3.1 将地图点插入对应的初始帧和当前帧

3.2 建立自地图点到关键帧的观测索引

​		从地图点类的Observation方法就可以找到该地图点在哪些关键帧上被观测到，在共视关系的更新中UpdateConnections用这个方法。在之后的BA优化中也依靠这个联系，足以在在g2o创建一条条edge。

3.3 计算地图点最具代表性的描述子

​		一般是选取规则如下：该地图点所在关键帧的所有观测特征点中，谁的描述子与其他所有描述子的距离总和最小，谁就是那个代表性的描述子。

3.4 计算地图点平均观测方向和深度

​		以后如果有新的关键帧建立地图点的共视关系了，可以根据这个作为判断是否为错误匹配的判断依据。

3.5 地图点添加到地图中

​		关键帧和地图点都放到地图中，就可以用来做BA优化了。

## step 4 更新初始地图里关键帧的共视关系和父子关系

​		主要是创建KFcounter并维护mConnectedKeyFrameWeights、mvpOrderedConnectedKeyFrames和mvOrderedWeights数据和建立父子关系，**详见KeyFrame::UpdateConnections()**

| KF的成员变量                   | 类型                    | 意义                         |
| ------------------------------ | ----------------------- | ---------------------------- |
| *mConnectedKeyFrameWeights*    | std::map<KeyFrame*,int> | 当前关键帧的共视关键帧及权重 |
| *mvpOrderedConnectedKeyFrames* | std::vector<KeyFrame*>  | 共视权重>15的关键帧          |
| *mvOrderedWeights*             | std::vector< int>       | 共视权重>15的权重            |
| mpParent                       | KeyFrame*               | 父关键帧                     |

## step 5 全局BA优化，同时优化所有位姿和三维点

就是使用g2o框架，把初始化的两帧关键帧和相应地图点所构成的地图进行优化，优化变量是地图点和位姿两类。详见Optimizer::GlobalBundleAdjustemnt（）和BA_G-N_g2o.md 。

## Step 6 取场景的中值深度，用于尺度归一化

## step 7 将两帧之间的变换归一化到平均深度1的尺度下

## Step 8 把3D点的尺度也归一化到1

## Step 9 将关键帧插入局部地图，更新归一化后的位姿、局部地图点



# 六、跟踪之一：恒速运动模型跟踪位姿—Tracking::TrackWithMotionModel()

## 思路

​		大部分时间都用这个跟踪，只利用到了上一帧的信息。**一句话总结：使用上一帧的位姿和速度求出粗略的初始位姿，使用投影的方式和上一帧地图点特征匹配，把匹配关系和初始位姿投入BA仅优化位姿。**

1. 假设短时间内相邻帧物体处理匀速运动状态，可以使用上一帧的速度和上一帧的位姿计算出当前帧位姿的初始值。（单目是这样，如果是双目&RGBD在UpdateLastFrame();里做的事情有区别）
2. 使用上一帧观测到的地图点 通过当前帧的位姿初始值投影到当前帧，寻找一定范围内的最优匹配。（在这里单目和双目&RGBD也有区别）
3. 用当前帧2D特征点和3D地图点的匹配关系，调用g2o构建图优化，仅优化当前帧的位姿。
4. 最后剔除一下外点，对跟踪到的地图点和特征匹配对的个数 计数，数量达到一定标准即为跟踪成功。

## step 1 获得上一帧的姿态，并根据上一帧的速度得到当前帧的初始位姿

这里存在两个问题：

1. **SLAM数据库中只存储了每个关键帧的姿态，那上一帧的姿态如何获得呢？**

​        在tracking线程中，每成功跟踪一帧，就会计算当前帧和其参考关键帧的相对位姿Tcr，把Tcr和关键帧分别储存在mlRelativeFramePoses和mlpReferences列表中。

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230921215228963.png" alt="image-20230921215228963" style="zoom: 80%;" />

​        由于关键帧的信息是记录在数据库中的，因此可以通过 上一帧与其参考KF的相对位姿*上一帧参考KF的位姿获得上一帧的位姿。在单目传感中，通过UpdateLastFrame();函数获得了上一帧的姿态。
$$
mLastFrame.mTcw =  Tcr * KF->GetPose()
$$

2. **上一帧的速度是什么？如何利用速度求解当前帧的初始姿态呢？**

​        速度（mVelocity），本质上就是记录了某一帧与其上一帧(无论是否是关键帧)的相对位姿。当求解了当前帧位姿后，只需求解与上一帧的相对位姿即可。mVelocity不像mlRelativeFramePoses和mlpReferences，速度是一个流动的变量而非容器，在当前帧的位姿还未求解时，mVelocity总是代表着上一帧与上上帧的相对位姿，也就是上一帧的速度，主要用来在恒速模型中计算当前帧的初始值。
$$
mVelocity = mCurrentFrame.mTcw * LastTwc;
速度=当前帧位姿乘以上一帧位姿之逆
$$
​        当前帧的初始姿态，使用速度 * 上一帧姿态计算。

```c++
mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
```

## step 2 特征匹配，使用3D点投影的方式，如果匹配点<20个则扩大搜索半径

​        使用LastFrame的地图点的投影到当前帧上，根据地图点对应LastFrame的特征点octive找对应FeatureArea范围内，将该范围内的当前帧特征点作为候选点。使用地图点的代表性描述子计算与这些候选点之间的距离，找到最优匹配并经过特征方向直方图的筛选，通过后注册当前帧特征点和地图点的连接。

详见matcher.SearchByProjection()

## step 3 使用g2o，仅优化当前帧的位姿

​		仅优化位姿，请详见（当前帧位姿优化—Optimizer::PoseOptimization(Frame)），这里和根据参考帧跟踪位姿Tracking::vTrackReferenceKeyFrame() 一样，都是使用poseOptimization函数，只优化了位姿而不优化地图点。

## step 4 剔除优化过程的外点，统计成功观测的匹配数量和地图点数量

​		优化过程中被判定为外点的特征点，将被剥夺地图点，统计经历匹配和优化过程后当前帧成功观测到的地图点数量，nmatchesMap>=10认为跟踪成功。如果是仅跟踪状态mbOnlyTracking，那么更关心匹配成功的点数量，nmatches>20认为跟踪成功。

### 涉及的关键变量（Track、KF、MP）

| Track成员变量        | 类型            | 意义                                                         | 描述                                                         |
| -------------------- | --------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| mVelocity            | cv::Mat         | 上一帧的速度(上一帧相对于上上帧的位姿)，用于恒速运动跟踪中，速度*上一帧的位姿得到当前帧的初始估计 | 速度是一个流动的变量而非容器，在当前帧的位姿还未求解时，mVelocity总是代表着上一帧与上上帧的相对位姿 |
| mlRelativeFramePoses | list< cv::Mat>  | 存储所有帧相对于参考关键帧位姿Tcr的历史记录                  | 配合mlpReferences可以记录所有帧的位姿                        |
| mlpReferences        | list<KeyFrame*> | 存储每个相对位姿关联的参考关键帧（就是mlRelativeFramePoses中的相对位姿是相对谁的） | 配合mlRelativeFramePoses可以记录所有帧的位姿                 |
| mLastFrame           | Frame           | 上一帧                                                       | 在恒速运动跟踪中，用于访问上一帧的特征点、位姿和地图点数据。 |

**mlRelativeFramePoses和mlpReferences两个变量可以记录SLAM系统中所有帧的位姿**。

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230921185026762.png" alt="image-20230921185026762" style="zoom: 50%;" />

| F、KF和地图点变量      | **类型**                   | **意义**               | **描述**                                                     |
| ---------------------- | -------------------------- | ---------------------- | ------------------------------------------------------------ |
| mLastFrame.mTcw        | cv::Mat                    | 上一帧的位姿           | 这里访问了Frame类的成员变量mTcw，用于获得上一帧的位姿。      |
| LastFrame.mvKeys       | std::vector< cv::KeyPoint> | 上一帧的特征点向量     | 索引代表特征点的索引，可以用来访问特征点的金字塔层数octave、坐标pt等。 |
| LastFrame.mvpMapPoints | std::vector<MapPoint*>     | 上一帧的地图点指针向量 | 索引代表特征点的索引，如果该特征点有对应的地图点，那么值为该地图点指针；如果该特征点不对应地图点，那么值为NULL。 |



# 七、跟踪之二：根据参考帧跟踪位姿——Tracking::vTrackReferenceKeyFrame

## 思路

​		在（（上一帧速度==0 或 刚完成重定位） 和 恒速模型跟踪失败）后使用。**一句话总结就是：使用上一帧的位姿作为初始位姿，使用词袋加速和参考关键帧特征匹配，把初始位姿和匹配关系投入BA仅优化位姿。**

1.尝试和参考关键帧做匹配，使用 Bag of words(BoW)的正向索引加速特征匹配。

2.使用**上一帧的位姿作为初始位姿**（即和上一帧是一个点，比较粗糙），使用g2o只优化当前帧的位姿。使用卡方阈值剔除外点。

3.匹配点<15个，优化后跟踪到的地图点>=10个才算跟踪成功。

## step 1 计算当前帧的词袋表示，得到正向索引

这一步主要是通过transform函数，把当前帧的所有特征点，转化为词袋表示，具体为两个std::map类型的数据。

其一：是BowVec，他表示一帧所有特征点按照树索引后，得到的所有wordID和权重。BowVec在后面可以用来构造逆向索引，作用于重定位和回环。

其二：是FeatureVec，也就是正向索引；他表示一帧所有特征点按照树索引时，指定某一层，获得的该层每个node下，该帧特征点的索引。FeatureVec可以用来加速和另外一帧的特征点匹配，一般来说我们指定第二层，这样只需要寻找每个相同node下的特征点两两匹配即可。

**详见：DBoW词袋-加速匹配-重定位**

```c++
/tracking中引用，计算当前帧的Bow
void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        // 将描述子mDescriptors转换为DBOW要求的输入格式
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);

        // 将特征点的描述子转换成词袋向量mBowVec以及特征向量（正向索引）mFeatVe
        mpORBvocabulary->transform(vCurrentDesc,  //当前的描述子vector
                                   mBowVec,       //输出，词袋向量，记录的是单词的id及其对应权重
                                   mFeatVec,      //输出，记录node id及其对应的图像 feature对应的索引 std::map<NodeId, std::vector<unsigned int> >
                                   4 );           //表示从叶节点向前数的层数（在正向索引时指定的层数＝Ｌ－４）
    }
}

```

## step 2 和参考帧特征匹配，利用正向索引加速

​		基于第一步得到的正向索引FeatureVec，对每个相同node下的参考帧和当前帧特征点计算距离匹配，找到最优的匹配之后会赋予当前帧特征点的地图点。这里SearchByBoW()和在初始化时特征匹配函数SearchForInitialization()有点相似，相同点是都使用最优和次优比例阈值 和 特征点角度差直方图主流三大方向用来剔除匹配点对，但初始化的时候没有用到BoW加速，而是使用了GetFeatureArea的形式用来加速。但两者在加速的思想很相似，都是通过区域的划分，避免了O(N^2)的计算复杂度。使用DBoW时，计算复杂度的变化如下，K代表词袋的第一层聚类个数，L表示当前用于搜索的node的层数。**详见：DBoW词袋-加速匹配-重定位 和 ORBmatcher::SearchByBoW()**

$$
O(N^2) -> O(N^2/K^L)
$$

## step 3 使用上一帧的位姿+ 参考帧特征匹配 进入g2o位姿优化

​		仅仅优化位姿而不优化3D点，把该帧的pose作为唯一的顶点，2D点作为观测值，2D-3D重投影误差使用Huber核函数作为损失。优化结束后根据投影误差和卡方阈值的比较区分内点和外点。**详见：Optimizer::PoseOptimization()**

## step 4 剔除优化过程的外点，统计成功观测的地图点数量

优化过程中被判定为外点的特征点，将被剥夺地图点，统计经历匹配和优化过程后当前帧成功观测到的地图点数量，数量>=10认为跟踪成功。

### 涉及的关键变量（Track、KF、MP）

| Track成员变量         | 类型                 | 意义         | 描述                                                         |
| --------------------- | -------------------- | ------------ | ------------------------------------------------------------ |
| mpReferenceKF         | KeyFrame*            | 参考关键帧   | 参考关键帧跟踪使用参考关键帧的信息，参考关键帧在每一次跟踪局部地图和插入新关键帧时被更新 |
| mLastFrame.mTcw       | cv::Mat              | 上一帧的位姿 | 参考关键帧使用上一帧的位姿作为该帧初始位姿                   |
| **F、KF和MP成员变量** | **类型**             | **意义**     | **描述**                                                     |
| mFeatVec              | DBoW2::FeatureVector | 正向索引     | 使用参考关键帧和当前帧的正向索引加速匹配，关键帧的           |
|                       |                      |              |                                                              |

# 八、跟踪之三：重定位跟踪位姿：Tracking::Relocalization()

## 为什么要重定位

​        假设相机运动不符合恒速运动模型，那么使用恒速运动模型发现跟踪点不够，跟丢了。于是这个时候会使用参考关键帧跟踪，参考关键帧使用位姿比较准确的关键帧来跟踪，结果发现还是没有跟踪上。于是就要使用重定位。

​        重定位跟踪通过逆向索引寻找最有可能在附近的N个关键帧，然后一一去做正向索引加速的匹配筛选出匹配点数目较多的，作为有效候选帧。重定位跟踪中，由于没有恒速模型和参考关键帧方法中的位姿初始值，所以需要使用RANSAC+EPnP的方式计算，目的是尽可能的算出一个较好的初始值，投入非线性优化的时候，能够更大概率收敛到全局最优。跟踪地图点。

## 思路

​		上一次跟踪的丢失的时候使用，mstate==Lost，利用到了某一个相似候选帧的信息，用于帮助当前帧跟踪地图点。**一句话总结：使用某一相似关键帧的匹配关系，用RANSAC+Epnp计算位姿初始值，并做BA优化位姿。**详细一点是：使用逆向索引找到候选相似关键帧集合，使用正向索引匹配筛选出有效的相似关键帧集合，遍历这些有效关键帧，利用匹配关系逐个用RANSAC+Epnp计算位姿初始值，再使用BA仅优化位姿。

### 重定位明明最终只会用到一个相似关键帧的信息，可为什么要找这么多帧呢？

​		我认为是为了最大可能的争取重定位成功，既然上一帧和参考关键帧都不能帮助当前帧成功跟踪，那说明当前帧的位置或运动方式和之前最近的几帧有很大区别，所以要尽可能的多找一些看起来和当前帧相似的关键帧，都拿来试一试，增大重定位成功的概率。

## step 1 使用DBow逆向索引，找到与当前帧最相似的候选关键帧集

​		计算当前帧的DBoW逆向索引，并以共享wordID数目、相似性分数等方式找到最相似的N个候选关键帧集，用来之后的匹配和位姿跟踪。***相关函数：DetectRelocalizationCandidates()***    

## step 2 DBow正向索引匹配，筛选出有效候选相似关键帧，并逐一初始化PnP求解器

​		遍历候选关键帧集，做DBow正向索引匹配，匹配点>15 作为有效候选帧，使用当前帧和该有效候选帧的匹配关系为其创建PnPsolver对象并设置RANSAC参数。

***相关函数：matcher.SearchByBoW(pKF,mCurrentFrame,)**   ，正向索引加速的特征匹配*

​                   ***pSolver = new PnPsolver()**    ，为该帧初始化PnP求解器对象*

​                  ***SetRansacParameters()**      ，设置RANSAC参数*

## step 3  EPnP线性初始值+BA非线性优化+根据内点数情况补充投影

​		逐个遍历有效候选帧，基于特征匹配得到的2D-3D对应关系，使用Epnp线性计算位姿作为初始值，内点数>mRansacMinInliers的话，用内点匹配使用g2o做非线性优化（仅优化位姿）。非线性优化之后根据内点数量，决定是数量太少放弃该帧进行下一帧 **或** 数量中等补充投影点再次非线性优化 **或** 数量够了直接返回。最多有两次补充投影匹配的机会，如果还是没有50个跟踪点，就换下一个候选帧重新进入step3，如果达到了50个跟踪点，就算做重定位成功，返回ture给mOb。

***相关函数：PnPsolverSolver::iterate()**     ，执行RANSAC+EPnP 和 refine()*

​                  ***Optimizer::PoseOptimization(&mCurrentFrame)**   g2o框架仅优化位姿*

### 3.1若优化完内点充足[50 - ~]，则直接返回位姿，不再使用其他的候选帧；若优化完内点较少[10-50)，使用候选帧投影补充新的匹配点；若优化完内点太少[~ - 10]，则放弃该帧使用下一个候选帧进入step3。

​		若需要候选帧投影补充新的匹配点，则使用SearchByProjection()函数用优化后的位姿投影匹配。对补充之后的总匹配点，再使用g2o做BA优化（仅优化位姿）。**然后再根据这次优化之后的内点数量再次决定，若优化完内点充足[50 - ~]，则直接返回位姿，不再使用其他的候选帧；若优化完内点较少[30-50)，使用候选帧投影补充新的匹配点；若优化完内点太少[~ - 30]，则放弃该帧使用下一个候选帧进入step3。**

***相关函数：matcher2.SearchByProjection()** ， 给定位姿将参考帧观测到的地图点投影到当前帧上，在一定 Area区域计算描述子进行特征点和地图点匹配。*

### 涉及的成员变量

| Tracking类         | 类型         | 意义             | 补充说明                                                     |
| ------------------ | ------------ | ---------------- | ------------------------------------------------------------ |
| mnLastRelocFrameId | unsigned int | 上一个重定位帧ID | 重定位成功后设为当前重定位帧，某个帧距离重定位的远近，影响着一些阈值，相对来说跟踪的要求更加严格一些。因此，在后续作为一种判据：选择是否使用参考关键帧跟踪、局部地图跟踪是否成功和是否插入关键帧的判据 |
|                    |              |                  |                                                              |



# 九、跟踪之补充优化：局部地图跟踪—— Tracking::TrackLocalMap()

## 为什么要跟踪局部地图呢？

​        局部地图是对于当前帧来说的，因为采用三个方法跟踪位姿时，只利用到了参考关键帧、上一帧或是某一个相似关键帧的信息，仅仅根据一帧的匹配关系和位姿先验优化位姿不是特别准确，因此该方法是三个跟踪位姿流程的后处理，旨在利用更多匹配关系优化位姿。

## 思路

​		基于使用三大跟踪已经跟踪成功的地图点，借助地图点的观测关系更新局部地图，寻找当前帧的两级共视关系将两级共视帧都作为局部地图内的关键帧(<80)，将使用所有局部关键帧的地图点匹配关系和跟踪流程中得到的位姿初始值再次进行非线性BA优化(仅位姿)。**一句话总结：基于已经跟踪成功的地图点，使用观测索引找到二级共视关键帧，使用两级共视关键帧的地图点投影匹配关系作为补充，BA优化当前帧的位姿。**

## step1 更新局部地图 

![img](https://img-blog.csdnimg.cn/58b2d5a3b5ca498fa1606196043e1406.png)

​		在三大跟踪方法中已经成功跟踪到一部分地图点，从当前地图点的观测关系中找到两级共视关键帧，把两级共视关键帧和它们的所有地图点都注册到局部地图中，使后续的位姿优化用到更多匹配关系，更加准确。**详见Tracking::UpdateLocalMap()**

### **1.1更新局部关键帧和参考关键帧。**

​		通过*GetObservations()*获得当前帧的共视帧作为一级局部关键帧；对于一级局部关键帧来说，找到其子关键帧、父关键帧和权重排名前10的共视帧作为二级局部关键帧；把一级二级都注册为局部关键帧。把一级局部关键帧中权重最高的帧作为当前帧的参考关键帧。**详见 Tracking::UpdateLocalKeyFrames()**

### **1.2更新局部地图点**

​		把局部关键帧的所有地图点作为局部地图点，**详见Tracking::UpdateLocalPoints()**

## step2 局部地图点投影到当前帧寻找匹配

​		在跟踪局部地图中，找到局部关键帧和地图点之后，需要补充匹配关系，因为已经有初始位姿了，所以使用局部地图点投影的方式补充匹配，之前已经跟踪到的地图点就不用重新进行匹配。并且由于没有使用其他的帧，也没有统计特征方向差的直方图。但是和之前恒速跟踪和重定位跟踪补充环节的投影匹配思想大差不差，思想都是基于初始位姿采用投影的方式加速匹配。**详见Tracking::SearchLocalPoints()**

## step3 使用局部地图的补充匹配关系，BA非线性优化位姿

​		使用跟踪环节优化的位姿作为初始位姿，用更多的匹配点BA仅优化位姿。

## Step4 更新当前帧的地图点被观测程度，并统计跟踪局部地图后匹配数目

**4.1更新当前帧的地图点被观测程度：**

更新mnFound：表示观测到该地图点的帧数量，通过mCurrentFrame.mvpMapPoints[i]->IncreaseFound()访问并增加mnFound的值。

**4.2统计跟踪局部地图后的匹配数目：**

mnMatchesInliers：跟踪局部地图之后的所有内点数目（包括跟踪流程的内点），储存在了mnMatchesInliers变量中。这个变量用于决定跟踪局部地图是否成功。

## Step 5 根据跟踪匹配数目及重定位情况决定是否跟踪成功

如果是刚重定位后的30帧以内，需要有50个内点数量才算是跟踪局部地图成功。

如果是刚重定位后的30帧外，那么有30个内点就算是跟踪局部地图成功。

### 涉及的成员变量

| Tracking类         | 类型                   | 意义                                 | 补充说明                                                     |
| ------------------ | ---------------------- | ------------------------------------ | ------------------------------------------------------------ |
| mvpLocalMapPoints  | std::vector<MapPoint*> | 局部地图点                           | 初始化时把前两帧三角化的内点作为局部地图点，之后在跟踪局部地图环节中更新 |
| mvpLocalKeyFrames  | std::vector<KeyFrame*> | 局部关键帧                           | 初始化时把前两帧作为局部关键帧，之后在判断关键帧中更新       |
| mnMatchesInliers   | int                    | 这一轮跟踪最终得到的内点数量         | 在跟踪局部地图之后统计得到的跟踪成功的内点数量，用于判断跟踪局部地图是否成功，和判断是否插入关键帧的一个指标。 |
| mnLastRelocFrameId | unsigned int           | 上一个重定位帧ID                     | 重定位成功后设为当前重定位帧，某个帧距离重定位的远近，影响着一些阈值，相对来说跟踪的要求更加严格一些。因此，在后续作为一种判据：选择是否使用参考关键帧跟踪、局部地图跟踪是否成功和是否插入关键帧的判据 |
| mpReferenceKF      | KeyFrame*              | 参考关键帧                           | 更新局部地图时把最大共视关键帧设为参考关键帧                 |
| **MapPoint类**     | **类型**               | **意义**                             | **补充说明**                                                 |
| mbTrackInView      | < bool >               | 搜索匹配时是否需要被投影的标记       | 在跟踪局部地图中，找到局部关键帧和地图点之后，需要补充匹配关系，当前帧之前已经跟踪到的地图点则不需要在进行后续的投影匹配环节，这个标记就用来标记该点在后面搜索匹配时不被投影，因为已经有匹配了。SearchLocalPoints()中 |
| mnLastFrameSeen    | < int >                | 该地图点最新被观测到时，是哪一帧的ID | 也用来判断是否需要去做补充匹配。SearchLocalPoints()中        |
| nObs               | < int >                | 观测到该点的相机数目                 | 单目+1，双目或RGB-D则+2                                      |
| mnFound            | < int >                | 观测到该点的帧数                     | 用于表示地图点被观测程度。                                   |



# 十、检查是否插入关键帧——NeedNewKeyFrame()

## 为什么要插入关键帧？

​        跟踪线程的对象，是相机的每一帧。但建图和回环线程的对象则是关键帧。跟踪线程的任务除了跟踪每一帧的位姿和地图点之外，还需要选出关键帧，利用关键帧的信息增删地图点，全局优化和回环。

​        总体来说，关键帧是一小群普通帧的代表，代表着这一群帧的特征点、地图点和位姿区间。因此，关键帧在VO中应该是均匀分布的，不能过于稠密，也不能过于稀疏，这样才能不空缺的迅速的表征整个地图。在跟踪环节，决定是否插入关键帧的要求比较宽松，为了避免遗漏，倾向于冗余的表示。因为在后面局部建图的环节中keyframeculling()还会剔除冗余的关键帧。

## 决定是否插入关键帧的标准  

​         跟踪环节决定是否插入关键帧有两个关卡，第一个关卡为拒绝关卡，第二个关卡为接受关卡。其中ABC为拒绝条件直接返回false，abcd为接受条件，满足后返回ture，不满足返回flase。不满足拒绝条件&&满足接受条件即可插入新关键帧。

### 拒绝关卡（前提）

- A：纯VO模式下，不插入关键帧 （mbOnlyTracking == ture）；
- B：如果局部地图线程被闭环检测使用，则不插入；
- C：该帧距离上一次重定位帧比较近，并且当前关键帧数目>fps，则不插入。

### 接受关卡

a条件为必要条件，bcd三个条件任意满足其一即返回ture否则为false：

- a：该帧经过位姿和局部地图跟踪后的内点数满足要求（该帧本身质量可以）&&  该帧跟踪到的地图点数<参考帧地图点数*比例系数（防止跟踪空缺，远离一定距离就插入）
- b：大于插入关键帧的最大间隔，距离上一次插入关键帧已经过去了mMaxFrames帧。（ps：说明很长时间没有插入关键帧了，为了避免遗漏，这时可以插入关键帧。mMaxFrames一般设为fps，表示每一秒的帧数）
- c：大于插入关键帧的最小间隔，且局部建图线程*localMapper*处于空闲状态。（ps：虽然没有到最大间隔，但是空闲状态说明可以插入，生产力不用白不用）
- d：该帧跟踪到的地图点数远小于参考帧 || 跟踪到的地图点和没有跟踪到的地图点差不多 （ps：该条件仅对于双目和RGBD传感情，该条件说明地图点数已经和之前相差很多了，急需补充关键帧）



# 十一、插入关键帧（单目）——CreateNewKeyFrame()

​		在决定插入关键帧之后，用当前帧初始化关键帧对象，并把该关键帧插入到列表 mlNewKeyFrames中，等待local mapping线程临幸。对于双目和RGBD来说，在创建关键帧的时候还为当前帧生成新的地图点，单目无操作。

| Tracking类       | 类型          | 意义           | 补充说明                                                   |
| ---------------- | ------------- | -------------- | ---------------------------------------------------------- |
| mnLastKeyFrameId | unsigned int  | 上一个关键帧ID | 插入关键帧后把该关键帧ID设为上一个关键帧ID，用于下一轮跟踪 |
| mpLastKeyFrame   | KeyFrame*     | 上一个关键帧   | 插入关键帧后把该关键帧设为上一个关键帧，用于下一轮跟踪     |
| mpReferenceKF    | KeyFrame*     | 参考关键帧     | 插入关键帧后把参考关键帧设为该关键帧                       |
| mpLocalMapper    | LocalMapping* | 局部地图       | 把关键帧插入到局部地图中                                   |
|                  |               |                |                                                            |

# 十二、Tracking流程的关键问题

## 12.1 地图点的创建与删除时机

**创建**

- 初始化成功后：初始化中三角化的内点作为初始地图点；
- 每次新插入关键帧（双目/RGBD）：在创建关键帧的时候为当前帧生成新的地图点；
- 局部建图环节的2.3：使用当前关键帧和他的一级共视关键帧三角化创建新地图点；
- 局部建图环节的2.4（融合，不算新建）：把2.3创建的新地图点放在两级共视关键帧中检查融合替换。

**删除**

- 局部建图环节的2.2：但凡每次有新建的地图点，都要备份在mlpRecentAddedMapPoint中接受检查，检查合格可留在地图中，不合格会被删除。（单双目初始化、双目新插入关键帧、局部建图的2.3）

## 12.2关键帧的更新

- 初始化成功后：将初始化的两帧设为关键帧；
- 每次新插入关键帧：CreateNewKeyFrame()；
- 局部建图环节：冗余关键帧的删除。

## 12.3参考关键帧的更新和使用

**参考关键帧的更新**：

有三处地方更新参考关键帧，分别为：

- 初始化成功后：初始化的第二帧暂时作为参考关键帧，用于初始化之后第一次参考关键帧跟踪的参考帧。
- 每一次的局部地图跟踪：把当前帧共视程度最高的那一个关键帧设置为参考关键帧。（虽然一般来说最大共视关键帧都指向最近的一个关键帧）
- 每次新插入关键帧：会把这个关键帧设为参考帧（该关键帧相对之后的帧来说，是距离最近的关键帧。只有创建关键帧时更新）

**参考关键帧的使用**：

- 参考关键帧跟踪位姿时（看上一轮有没有新增关键帧，但两处更新的地方基本上都指向一个关键帧，也就是那个最新的关键帧）
- 跟踪局部地图时使用并且更新参考关键帧（总是最大共视关键帧）

## 12.4 哪些帧计算DBoW词袋，什么时候计算

​		一般来说，使用DBow的逆向和正向索引时，需要使用该帧的特征点描述子计算。用到逆向和正向索引的地方主要有以下几处：

- **参考关键帧跟踪时：计算当前帧的词袋向量正向索引。**ps：由于没有恒速模型带来的良好位姿初始值，无法使用投影的匹配方式，只能使用词袋加速匹配的方式。

- **重定位跟踪时：计算当前帧的词袋向量逆向和正向索引。**ps：逆向索引用来寻找两级相似关键帧，正向索引用来加速特征匹配。

- **局部建图时：计算关键帧的词袋逆向和正向索引。**局部建图环节处理新插入的关键帧时，就计算了该关键帧的词袋向量，之后参考关键帧和重定位跟踪中关键帧的词袋向量就是事先在此计算的。

## 12.5共视图什么时候更新？有什么用？

### 什么时候更新

- 初始化成功，建立初始地图时：建立初始地图把初始化的帧和三角化点作为关键帧和地图点放入地图中，并更新了共视关系。

- 局部建图时：局部建图环节处理新插入的关键帧中，在2.4步融合和2.6剔除冗余关键帧之后更新了共视图。

### 有什么用？（共视关键帧的使用）

- 用于关键帧按照共视权重寻找他的共视关键帧，如在跟踪局部地图、局部建图线程（新建地图点和地图点融合）环节都使用了共视关系去寻找某帧的一级或二级共视关键帧。

## 12.6每当新关键帧插入地图，新地图点建立，都需要做什么？

## 12.7跟踪线程和局部建图的数据交互在哪里？

​		每当Tracking流程中，插入关键帧时InsertKeyFrame()，局部建图线程就会来活了。Tracking流程中在单双目初始化、单双目插入关键帧时会使用InsertKeyFrame()。

## 12.8局部建图和回环检测的数据交互在哪里？
