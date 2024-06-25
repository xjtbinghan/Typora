[TOC]



# ORBextractor模块之成员变量

## 一、从Yaml配置文件中读入

| 成员变量            | 意义                             | 配置文件中的变量名       | 值   |
| ------------------- | -------------------------------- | ------------------------ | ---- |
| int  nfeatures      | 特征点数目                       | ORBextractor.nFeatures   | 1000 |
| double  scaleFactor | 图像金字塔相邻层<br />的尺度因子 | ORBextractor.scaleFactor | 1.2  |
| int  nlevels        | 图像金字塔层数                   | ORBextractor.nLevels     | 8    |
| int  iniThFAST      | 特征点提取的高阈值               | ORBextractor.iniThFAST   | 20   |
| int  minThFAST      | 特征点提取的低阈值               | ORBextractor.minThFAST   | 7    |

## 二、根据上述成员变量的值计算而来的，在Frame类中

| 成员变量                                   | 意义                              | 值                                                          |
| ------------------------------------------ | --------------------------------- | ----------------------------------------------------------- |
| std::vector<int> <br />mnFeaturesPerLevel; | 每层金字塔提取的特征点个数        | {60，73，87， 105，126，151，181，217}合为1000（nfeatures） |
| std::vector<int> <br />umax;               | 拟合半径为16的1/4圆的16个近似坐标 | umax[0]=16 uamx[1]=16 ······  uamx[15]=1                    |
| std::vector<float> <br />mvScaleFactor;    | 各层级的尺度系数                  | {1，1.2，1.44，1.728，2.074······}共八个                    |
| std::vector<float> <br />mvInvScaleFactor; | 各层级尺度系数的倒数              | {1，0.833，0.694 ，0.579······}                             |
| std::vector<float> <br />mvLevelSigma2;    | 各层级尺度系数的平方              | {1，1.44，2.074，2.986······}                               |
| std::vector<float> <br />mvInvLevelSigma2; | 各层级尺度系数平方的倒数          | {1，0.694 ，0.482 ，0.355······}                            |



# ORBextractor模块之成员函数

## 一、构造函数——ORBextractor::ORBextractor

### step 0 函数调用关系（暗线）

**Tracking线程初始化**的时候，初始化了ORB特征提取器。而system初始化的时候初始化了tracking线程。从而暗线是这样的：**system初始化—>Tracking初始化—>ORBextractor初始化**

```c++
//2.2 初始化 ORB 特征提取器对象
    mpORBextractorLeft = new   ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
```

### step 1 计算各金字塔层的尺度系数 与 各层级尺度系数的平方数组 及其倒数

通过尺度系数和金字塔层数，计算出每一层的金字塔图像对应的尺度系数以及平方、倒数等数据

### step 2 计算每个层级期望提取的特征点数量

按照尺度系数进行划分，分辨率越大的按比例提取更多的特征点，最下层的金字塔是原图。所有金字塔层共提取1000个特征点。

### step 3 初始化用于计算描述子的pattern变量

bit_pattern_31中包括256*4个数，其中每两个数表示一个坐标点。在ORBextractor的源代码中，用于计算描述子的变量bit_pattern_31_被写死，第三步使用指针和stl容器操作把bit_pattern_31的内容复制到pattern指针，数据类型为cv::point，表示2D点的数组。因此，pattern中共有512个2D点。

### step 4 计算一个半径为16的圆的第一象限近似坐标点

通过点的方式逼近1/4圆，并将坐标前存储在umax向量中，用于后续计算特征点主方向和描述子。

```c++
ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
         int _iniThFAST, int _minThFAST):
    nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
    iniThFAST(_iniThFAST), minThFAST(_minThFAST) 
{

    //step 1 计算各层级的尺度系数 与 各层级尺度系数的平方数组 及其倒数
    //数组长度==金字塔层数
    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<nlevels; i++)
    {
        
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;  
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }
    
    //各层级的尺度系数倒数 与 各层级尺度系数的平方倒数
    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

    //step 2 计算每个层级期望提取的特征点数量。根据特征提取的期望数量，这些特征点将在不同的金字塔层级中提取。
    //并保存在mnFeaturesPerLevel数组中[217, 181, 151, 126, 105, 87, 73, 60]
    mvImagePyramid.resize(nlevels);
    mnFeaturesPerLevel.resize(nlevels);
    float factor = 1.0f / scaleFactor;
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));
    int sumFeatures = 0;
    for( int level = 0; level < nlevels-1; level++ )
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);

    //step 3 初始化用于计算描述子的pattern变量
    //这里是使用指针和容器操作把bit_pattern_31_的内容复制到pattern指针中
    const int npoints = 512;
    const Point* pattern0 = (const Point*)bit_pattern_31_;
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

    // step 4 计算一个半径为16的圆的近似坐标
    // 解释为使用16个坐标点，拟合一个半径为16的四分之一圆弧，每个点的坐标。使用u v作为横纵坐标，当v坐标从0到半径时，u的坐标。

    umax.resize(HALF_PATCH_SIZE + 1);
    int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1); // cvfloor向下取整 cvRound四舍五入 cvCeil向上取整
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
    const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(sqrt(hp2 - v * v));

    // 根据对称性对称过来
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }
}
```



## 二、主函数——ORBextractor::operator()

即重载的operator()运算符，是对外的特征提取接口，其中调用关系和函数内容如下：

### step 0 函数调用关系（暗线）

在system初始化之后，图像数据进入了Tracking流程中，Tracking流程首先预处理图像，之后构造frame，在构造frame的时候，使用了ORB特征提取。以RGBD相机为例，暗线如下：

**system::TrackRGBD—>Tracking::GrabImageRGBD（step 2构造frame）—>frame初始化—>ORBextractor::operator()（ORB特征提取）**

在frame类初始化中调用了Frame::ExtractORB，其再使用(*mpORBextractorLeft)调用了ORBextractor类的算术重载运算符operator()，调用了主函数。这也是ORBextractor主函数唯一被调用的地方，也就是说，每一个图像在构造frame的过程中，首先进行ORB的特征提取。

(*mpORBextractorLeft)解读：mpORBextractorLeft表示ORB提取器的指针，星号表示指针所对应的类地址，外面加上（）括号表示该类的算术重载符，即调用operator()方法。

```c++
//frame构造函数中
/step 2 ORB 特征提取
    //提取灰度图像的orb特征
    ExtractORB(0,imGray);
```

```c++
//frame的ExtractORB函数
void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}
```



### step 1 检查图像有效性

首先检查是否有图像，确保图像是单通道的8位无符号整数图像。确认之后进行下面的流程。

### step2 构建图像金字塔

调用ComputePyramid（）函数构建图像金字塔

### step3 计算特征点并进行八叉树筛选

调用 ComputeKeyPointsOctTree（）函数对特征点进行八叉树筛选

### step4 遍历金字塔每一层图像计算描述子

获得金字塔每一层的图像、特征点向量和特征点个数；将图像高斯去噪后，调用computeDescriptors（）函数计算每一层图像的描述子。

```c++
void ORBextractor::operator()( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints,
                      OutputArray _descriptors)
{ 
    //step1 检查图像有效性
    //检查是否有图像
    if(_image.empty())
        return;
    //将输入图像转换为 OpenCV 的 Mat 对象。然后使用
    Mat image = _image.getMat();
    //确保输入图像是单通道的8位无符号整数图像
    assert(image.type() == CV_8UC1 ); //assert的用法是判断括号内的条件是否满足，为真则继续进行，为假则报错

    // step2 构建图像金字塔（缩放+padding）
    ComputePyramid(image);

    // step3 计算特征点并进行八叉树筛选
    // 创建一个嵌套的 KeyPoint 向量，用于存储各个金字塔层级的特征点。
    vector < vector<KeyPoint> > allKeypoints;
    ComputeKeyPointsOctTree(allKeypoints);
    //ComputeKeyPointsOld(allKeypoints);

    
    Mat descriptors;
    int nkeypoints = 0;
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += (int)allKeypoints[level].size();
    if( nkeypoints == 0 )
        _descriptors.release();
    else
    {
        _descriptors.create(nkeypoints, 32, CV_8U);
        descriptors = _descriptors.getMat();
    }

    _keypoints.clear();
    _keypoints.reserve(nkeypoints);

    //step4 遍历金字塔每一层图像计算描述子
    int offset = 0;
    for (int level = 0; level < nlevels; ++level)
    {
        //获得每一层图像的特征点向量和特征点个数
        vector<KeyPoint>& keypoints = allKeypoints[level];
        int nkeypointsLevel = (int)keypoints.size();

        //如果当前层的关键点总量==0则跳出此次循环
        if(nkeypointsLevel==0)
            continue;

        // preprocess the resized image
        //获得对应金字塔层的图像
        Mat workingMat = mvImagePyramid[level].clone();
        //计算描述子前先进行高斯模糊（减少噪声和细节）
        GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

        // 计算每一层特征点的描述子
        Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        computeDescriptors(workingMat, keypoints, desc, pattern);

        offset += nkeypointsLevel;

        // Scale keypoint coordinates
        if (level != 0)
        {
            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
            for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                 keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
                keypoint->pt *= scale;
        }
        // And add the keypoints to the output
        _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
    }
}
```



## 三、构造图像金字塔——ComputePyramid

### step 0 函数调用关系（暗线）

在ORB主函数的step 2 中构建了图像金字塔。

**ORBextractor::operator()（ORB特征提取 step 2）—>ComputePyramid()**

### step 1 创建循环

按照循环一层层构建金字塔，并存放到mvImagePyramid向量之中，金字塔实际上是每一层缩放+padding。

### step 2 创建缩放和padding后的空图像temp

按照对应层，创建该层缩放和padding之后空间大小的空mat temp，用于第三步被上一层复制粘贴并存放到mvImagePyramid[level]中

### step 3 下采样构建图像金字塔

**如果是第一层**，那么直接将将原始图像复制到 temp 中，并添加边界像素，使用 BORDER_REFLECT_101 边界类型。copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,BORDER_REFLECT_101); 

**如果不是第一层**，那么使用下采样的方法再使用copyMakeBorder方法构建下采样和padding后的图像保存在mvImagePyramid[level]中

```c++
//构造图像金字塔
void ORBextractor::ComputePyramid(cv::Mat image)
{
    //step 1 创建循环将每一层图像放入mvImagePyramid中
    for (int level = 0; level < nlevels; ++level)
    {
    //step 2 创建缩放和padding后的临时图像temp
        float scale = mvInvScaleFactor[level];
        //按照该层尺度缩放后的宽高
        Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
        //缩放后加上padding的19个像素
        Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
        //创建了一个临时图像 temp，大小为 wholeSize，类型与输入图像 image 相同
        Mat temp(wholeSize, image.type()), masktemp;
        //mvImagePyramid是一个矩阵数组，每一个矩阵存储相应层的图像mat，数组个数对应金字塔层数。
        //这表示将临时的调整大小后的图像 temp 的指定区域赋值给图像金字塔的某一层
        mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

    // step 3 下采样构建图像金字塔
        if( level != 0 )
        {
            //如果当前层级不是第一层，将上一层级的图像通过插值方法进行重采样，生成当前层级的图像
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);
            //将图像复制到 temp 中，并在图像周围添加边界像素，用于处理图像边缘的情况。
            //这是通过调用 copyMakeBorder 函数实现的，边界类型为 BORDER_REFLECT_101+BORDER_ISOLATED
            copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101+BORDER_ISOLATED);            
        }
        else
        {
            //如果当前层级是第一层，将原始图像复制到 temp 中，并添加边界像素，同样使用 BORDER_REFLECT_101 边界类型。
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,BORDER_REFLECT_101);            
        }
    }

}
```



## 四、八叉树——DistributeOctTree

## 五、计算特征点描述子——computeDescriptors

## 六、计算特征点主方向——computeOrientation

## 七、提取特征点八叉树筛选——ComputeKeyPointsOctTree

