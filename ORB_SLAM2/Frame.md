[TOC]

![img](https://pic1.zhimg.com/80/v2-c3d355e322ec1bbf0dcdd343a0b3839c_720w.webp)

# Frame模块之成员变量

一共有这么多的成员变量，看完代码后一个个解释：

```c++
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
```

frame构造的时候直接传入的变量，上面还有很多是再frame代码中通过计算和调用其他类得来的。



| 成员变量                                   | 意义                              | 值                                                          |
| ------------------------------------------ | --------------------------------- | ----------------------------------------------------------- |
| std::vector<int> <br />mnFeaturesPerLevel; | 每层金字塔提取的特征点个数        | {60，73，87， 105，126，151，181，217}合为1000（nfeatures） |
| std::vector<int> <br />umax;               | 拟合半径为16的1/4圆的16个近似坐标 | umax[0]=16 uamx[1]=16 ······  uamx[15]=1                    |
| std::vector<float> <br />mvScaleFactor;    | 各层级的尺度系数                  | {1，1.2，1.44，1.728，2.074······}共八个                    |
| std::vector<float> <br />mvInvScaleFactor; | 各层级尺度系数的倒数              | {1，0.833，0.694 ，0.579······}                             |
| std::vector<float> <br />mvLevelSigma2;    | 各层级尺度系数的平方              | {1，1.44，2.074，2.986······}                               |
| std::vector<float> <br />mvInvLevelSigma2; | 各层级尺度系数平方的倒数          | {1，0.694 ，0.482 ，0.355······}                            |
|                                            |                                   |                                                             |



| 成员变量            | 意义                                                         | 来源&值                                                      |
| ------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| mK                  | 相机内参矩阵                                                 | 来源于配置文件，由tracking传入                               |
| mbf                 | 基线*焦距                                                    | 来源于配置文件，由tracking传入                               |
| mThDepth            | 双目或RGBD计算出的特征点深度和基线的倍数<br />超越该值说明深度不准不使用 | 来源于配置文件，由tracking传入                               |
| mDistCoef           | 畸变矫正系数矩阵                                             | 由tracking计算并传入                                         |
| mTimeStamp          | 时间戳                                                       | 由demo入口读取图像的时候传入，数据经过system::TrackRGBD->tracking::Grabimage->frame |
| mpORBextractorLeft  | （对象指针）初始化的ORB提取器                                | tracking初始化时初始化了ORB提取器，在tracking::Grabimage方法中使用<br />tracking::tracking->tracking::Grabimage->frame |
| mpORBextractorRight | （对象指针）初始化的ORB提取器                                | tracking初始化时初始化了ORB提取器，在tracking::Grabimage方法中使用<br />tracking::tracking->tracking::Grabimage->frame |
| mpORBvocabulary     | （对象指针）ORB词袋                                          | 来源于ORB词袋文件，system::system->tracking::tracking->tracking::Grabimage->frame |
| N                   | 特征点的数量                                                 |                                                              |
| mvKeys              | 特征点向量                                                   | 数据类型为cv::KeyPoint ，是一种特殊的专门表示x，y坐标的数据格式 |
| mvKeysUn            | 矫正后的特征点向量                                           | 数据类型为cv::KeyPoint ，是一种特殊的专门表示x，y坐标的数据格式 |
| mvbOutlier          | 这是一个标志数组，用于标记特征点是否被判定为外点             | vector<bool>(N,false)，N表示特征点的索引，false表示不为外点，ture表示为外点。初始化、三角化和优化过程中被判定为外点的特这个点将被注册为ture。 |
| mvpMapPoints        | 该帧的地图点，坐标系是世界坐标系。一般来说只有跟踪时匹配成功的帧才注册地图点。 | vector<MapPoint*>，向量的索引代表该帧的特征点索引，特征点和地图点是一一对应的 |



# Frame模块之成员函数

## 一、构造函数——Frame::Frame

### step 0 暗线

在每一张图像传入时，第一个经过的是构造frame过程，首先创建mCurrentFrame的对象表示当前帧。Frame初始化是从tracking流程的入口Tracking::GrabImageRGBD中调用的，如下是Tracking::GrabImageRGBD函数的部分，在这里面，先使用灰度图像数据、时间戳、深度图和ORB提取器、相机内参等等数据构造Frame，在构造了Frame之后，再进去Track的主流程中。

```c++
// step 3 使用生成的灰度图构造frame
mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
//step 4 进入Track（）函数进行tracking流程
Track();
```

frame的构造函数写了好几个，用来拷贝，不知道为什么要拷贝，但直接看下面的就行了。

```c++
//默认构造函数 什么都不做
Frame::Frame()
{}

//拷贝构造函数，用另一个Frame对象初始化新的对象。
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);  
}
```

以RGBD为例，构造函数主要有以下三步

### step 1 获取特征提取器相关参数（主要是金字塔的参数）

通过mpORBextractorLeft指针，调用ORBextractor类的方法（GetLevels();等等），这些方法获得了图像金字塔的相关参数。包括金字塔层数、尺度系数、每层的尺度系数向量、每层的尺度系数平方向量、每层的尺度系数倒数向量、每层的尺度系数平方的倒数向量这些成员变量；

### step 2 提取orb特征、矫正畸变、特征点和Depth的匹配与计算

![image-20230830164833856](/home/hanbing/.config/Typora/typora-user-images/image-20230830164833856.png)

**2.1 orb特征提取** 

使用tracking中初始化后的orb提取器提取图像特征点，这里通过ExtractORB(0,imGray）调用Frame::ExtractORB方法，通过 (*mpORBextractorLeft)算术重载的方式，调用ORBextractor类的主函数，对imGray进行特征点提取，这也是ORBextractor::operator()在整个环节中唯一使用的一次。

返回mvKeys,mDescriptors两个成员变量，分别表示特征点向量和对应的描述子。

int flag表示是否是双目，如果是双目，那么使用ORBextractorRight提取右图特征。

```c++
//step 2 ORB 特征提取
    //提取灰度图像的orb特征
    ExtractORB(0,imGray);

...
    
void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

//ORBextractor主函数
void ORBextractor::operator()( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints,OutputArray _descriptors)
```

**2.2 畸变矫正**

详见 畸变矫正——Frame::UndistortKeyPoints() 方法，将ORB提取的特征点通过opencv的畸变矫正的方法，矫正到正确的坐标上。

**2.3 左右目特征点匹配，计算特征点深度信息**

双目传感器详见：ComputeStereoMatches(); ，双目方法利用左右目的特征点，通过粗匹配、细匹配，找到匹配点，利用视差公式求解了特征点深度。

RGBD传感器详见：ComputeStereoFromRGBD(const cv::Mat &imDepth)，RGBD方法利用深度信息和视差公式，求解了右目特征点

### step 3 初始计算时需要的步骤

如果传入的是第一张图像，那么需要进行以下步骤：

![image-20230830172600071](/home/hanbing/.config/Typora/typora-user-images/image-20230830172600071.png)

3.1 计算图像的边界左上角、右上角、左下角和右下角。详见：Frame::ComputeImageBounds(imGray) 

3.2 获取元素网格倒数、图像边界宽高和相机参数等数据并存储

### step 4  将特征点分配到网格中

![image-20230830172539673](/home/hanbing/.config/Typora/typora-user-images/image-20230830172539673.png)

根组特征点矫正畸变后的位置将特征点分配到网格中。
详见（六、矫正后的特征点分配到网格中——Frame::AssignFeaturesToGrid）（七、判断特征点在哪个网格中——Frame::PosInGrid）

### 源码

```c++
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // 记录这是第几帧
    mnId=nNextId++;

 //step 1 获取特征提取器相关参数，主要是金字塔参数
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

 //step 2 提取orb特征、矫正畸变、特征点和Depth的匹配与计算
    //2.1提取灰度图像的orb特征
    ExtractORB(0,imGray);
    //提取出来的特征点数目
    N = mvKeys.size();
    //如果没有关键点，返回
    if(mvKeys.empty())
        return;
    
    //2.2调用Opncv的矫正函数矫正orb提取的特征点（去畸变）
    UndistortKeyPoints();

    //2.3从RGB-D图像计算立体匹配，获得特征点的深度，虚拟右目特征点
    ComputeStereoFromRGBD(imDepth);

    //初始化mvpMapPoints为一个具有N个空指针的MapPoint指针向量
    //static_cast将空指针转化为指向MapPoint的空指针，在初始化时，这些指针被设置为空，而不指向实际的 MapPoint 对象
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL)); 
    //标识异常关联的标志，初始化mvbOutlier为一个具有N个false值的布尔数组
    mvbOutlier = vector<bool>(N,false);

 //step 3 初始计算时 获取图像边界宽高和相机参数等数据并存储
    if(mbInitialComputations)
    {   
        //3.1 根据是否需要矫正畸变来计算图像边界左上角、右上角、左下角和右下角。
        ComputeImageBounds(imGray);

        //3.2 计算网格元素宽度和高度的倒数
        /*FRAME_GRID_COLS=64  FRAME_GRID_ROWS=48
        相当于把图像的宽高分为64*48个方格
        之后特征点的座标点只需乘以（mfGridElementWidthInv和mfGridElementHeightInv）就可以获得该特征点在那块方格中 */
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);
        
        //获取相机参数
        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;
        //只有第一帧进行这些计算、后面所有帧都不计算
        mbInitialComputations=false;
    }
    
    //获得基线
    mb = mbf/fx;
    //step 4 根据特征点矫正畸变后的位置将特征点分配到网格中
    AssignFeaturesToGrid();
}
```

## 二、畸变矫正——Frame::UndistortKeyPoints()

### step 0 调用关系

构造函数Frame::Frame中在ORB特征提取后调用

```c++
//调用Opncv的矫正函数矫正orb提取的特征点（去畸变）
    UndistortKeyPoints();
```

### step 1 判断是否有矫正系数

找校正系数矩阵（5,1）的第一个元素，如果第一个元素=0,说明没有矫正系数，那么不矫正，mvKeysUn=mvKeys;直接return；

### step 2 创建mat用于拷贝mvKeys的数据

创建N行2列1channel的mat 用于拷贝cv::points数据格式的mvKeys。

### step 3 cv::undistortPoints矫正畸变

首先mat=mat.reshape(2);把N行2列1channel的mat改为 把N行1列2channel。这样改是为了符合cv::undistortPoints的使用规范。
cv::mat::reshape（）的使用规则如下：

```c++
Mat::reshape(int cn, int rows=0) const
//cn: 表示通道数(channels), 如果设为0，则表示保持通道数不变，否则则变为设置的通道数。
//rows: 表示矩阵行数。 如果设为0，则表示保持原有的行数不变，否则则变为设置的行数。

```

cv::undistortPoints（）用于根据相机参数和观测到点坐标位置计算实际坐标位置，使用规则如下：

```c++
void cv::undistortPoints(InputArray src,
    OutputArray dst,
    InputArray cameraMatrix,
    InputArray distCoeffs,
    InputArray R = noArray(),
    InputArray P = noArray() )
//src 是特征点坐标cv::mat，shape为1xNx2或Nx1x2
//dst 是矫正后的坐标cv::mat，
//cameraMatrix 相机内参矩阵
//distCoeffs 矫正矩阵
//R参数 是用在双目里的，单目里置为空矩阵
//P矩阵值为空时，得到的结果的点坐标是相机的归一化坐标(x,y)，这时候数值就会明显很小；设置相机内参后可以得到正常消畸变后的像素坐标
```

### step 4 矫正后的坐标存入mvKeysUn

将矫正后的mat放入mvKeysUn成员变量中，其中包含一次数据格式的转换，mat 是 cv::Mat数据格式，mvKeysUn是cv::KeyPoint数据格式。

### 源码

```c++
//特征点矫正畸变
void Frame::UndistortKeyPoints()
{
    //step 1 判断是否有矫正系数
    //如果没有矫正系数，那么不矫正，mvKeysUn=mvKeys;
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys; //mvKeysUn表示矫正后的特征点，mvKeys表示还未矫正的特征点
        return;
    }

    // step 2 创建mat用于存储mvKeys的数据
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        //mvKeys是一个存储特征点的向量，其中每个元素都是一个cv::KeyPoint结构，包含特征点的各种信息，包括坐标。
        //将mvKeys的x，y坐标分别赋予在第一列和第二列中
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    //step 3 矫正畸变
    //这里reshape的用法如下cv::Mat::reshape(int new_cn, int new_rows = 0) const;cn代表改变后的通道，rows表示改变后的行。列自动计算
    //mat.reshape(2);将n行2列1channel 改变为了n行 1列 2 channel。每行具有两个通道，表示x和y坐标，这么改是为了符合cv::undistortPoints的使用规则
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    //改为原先的N行2列
    mat=mat.reshape(1);

    //step 4 矫正后的坐标存入mvKeysUn
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}
```

## 三、双目匹配——Frame::ComputeStereoMatches()

### Step 0 暗线（调用关系）

在以双目为传感器的Frame::Frame构造函数中调用，用来计算左右目特征点的匹配和深度。

![image-20230830151239610](/home/hanbing/.config/Typora/typora-user-images/image-20230830151239610.png)

### Step 1 建立特征点搜索范围对应表vRowIndices，将iR注册到相邻行中

创建一个二维向量vRowIndices，用来保存左目特征点所在行可能对应的右目特征点。

对于每一个右图特征点，基于该特征点所在的行和图像金字塔层数，计算出r表示"该行可能存在特征点的范围“，然后将vRowIndices中对应行上的向量中，都插入右图特征点的索引iR，作为在该行上的左图特征点的候选匹配集。

也就是说如果iR点的行数y=20, r =2 的话。那么在vRowIndices[18]、vRowIndices[19]、vRowIndices[20]、vRowIndices[21]、vRowIndices[22]上都有这个特征点索引，一行可以有多个右图特征点索引iR，每一个索引iR也可以分布在多个vRowIndices行中。

```c++
   //在右图像中遍历每个特征点 iR，然后根据特征点的位置计算其影响的行范围，并将该特征点的索引存储在 vRowIndices 中对应的行中。
    //获取右图的特征点数量
    const int Nr = mvKeysRight.size();
    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        //搜索时，对于特征点iR，以一种横带的方式再另一幅图上搜索与其匹配的点。横带的最上行是maxr，最下行是minr
        //会根据特征点iR所在金字塔层数对应的缩放系数，调整r的值，图像越大则对应的搜索范围越大
        //也就是说如果iR点的行数y=20,那么在vRowIndices[minr～maxr]上都有这个特征点索引，认为区间内都有这个特征点，用于在另一张图上搜索。
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);
        //把特征点索引iR插入vRowIndices对应行的向量中
        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }
```



### Step 2 粗匹配：每个特征点iL按照金字层级，描述子距离找出最优匹配iR

2.1 获得每一个左图特征点的金字塔层、x，y坐标

2.2 根据y坐标找到对应行的候选匹配集，在对iL和iR所在的（金字塔层是否相邻）以及（列是否在一定范围内）进行筛选。

2.3 计算符合条件的候选匹配点iR与iL的描述子距离（汉明距离），并找到最佳匹配（距离最近）。其中调用了ORBmatcher::DescriptorDistance()方法。

```c++
// step 2 ：粗匹配，对左目每个特征点iL，按照金字层级，描述子距离找出最优匹配
    // 注意：这里是校正前的mvKeys，而不是校正后的mvKeysUn
    // KeyFrame::UnprojectStereo和Frame::UnprojectStereo函数中不一致

    //vDistIdx 用于存储每个左图像特征点与其对应的最佳右图像特征点的距离和索引。
    //每个元素都是一个整数对，第一个整数表示距离，第二个整数表示特征点的索引。
    vector<pair<int, int> > vDistIdx;  //pair<int, int>是整数对格式
    vDistIdx.reserve(N);               //N是左图特征点的个数，预留了N个特征点的内存
    
    for(int iL=0; iL<N; iL++)
    {
        //对于每一个左图特征点（未矫正）找到其对应的金字塔层、x，y坐标
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;
        //对于每一个左图特征点，找到右图对应行上包含的右图特征点索引候选集
        //也就是说，上面vRowIndices已经了涵盖左图的每一行可能对应的右图特征点的索引
        //所以，对于左图的特征点，只需要查找该行vRowIndices包含的右图特征点iR索引即可，然后进行距离计算。
        const vector<size_t> &vCandidates = vRowIndices[vL];

        //vCandidates表示当前行对应的右图特征点索引
        if(vCandidates.empty())
            continue;
        
        //x的范围区间 即列的区间，要求左右图的特征点行和列都保持在一定的范围内
        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue; 

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;
        
        //当前iL的描述子
        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints

        //对候选集中的每个右相机特征点，计算其与左相机特征点的描述子距离，并找到最佳匹配。
        //遍历候选集中的匹配特征点索引
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            //判断右图特征点的金字塔层是否与左图特征点相近
            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;


            const float &uR = kpR.pt.x;
            //判断右图特征点的水平位置是否在合理范围内
            if(uR>=minU && uR<=maxU)
            {
                //当前iR的描述子
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                //计算两个描述子之间的距离
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                //遍历vCandidates中的候选匹配集，找出最优匹配，并得到最优匹配的距离和索引
                if(dist<bestDist) 
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }
```

### step 3 细匹配：滑动窗口匹配和亚像素的抛物线拟合匹配

对上一步得到的最优匹配点进行亚像素的匹配，主要通过滑动窗口搜索和SAD抛物线拟合等方法，没有认真看。

最后根据双目时差公式计算出了深度，以及计算出了匹配点的横纵坐标。

3.1 如果最优匹配符合阈值要求，进行细匹配

3.2 滑动窗口细匹配，以iL和iR为中心，取5x5的窗口滑动，以窗口内的像素L1距离寻找精确匹配

3.3 将滑动窗口找到的精确匹配点加上左右两个点，以点的坐标为自变量，点对应的L1距离为值， 构建一个二次曲线，取曲线的最小值，即为精确匹配的点。最小值很可能是个小数，用来计算Depth更加精确，因此名为亚像素的匹配。

```c++
//step 3 细匹配：如果最优匹配在阈值内（bestDist<thOrbDist），则进行滑动窗口匹配和亚像素的抛物线拟合匹配
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            //获得该特征点所在金字塔层的图像分辨率，和特征点坐标
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            //滑动窗口匹配，根据匹配点周围5 x 5的窗口像素距离寻找精确匹配
            const int w = 5;  //// 滑动窗口的大小11*11 该窗口取自resize后的图像
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            //滑动窗口的滑动范围为（-L, L）,提前判断滑动窗口滑动过程中是否会越界
            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                // 横向滑动窗口
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                // 窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1); // 一范数，计算差的绝对值

                if(dist<bestDist)
                {
                    bestDist =  dist;   // SAD匹配目前最小匹配偏差
                    bestincR = incR;    // SAD匹配目前最佳的修正量
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            // 做抛物线拟合找谷底得到亚像素匹配deltaR
            // (bestincR,dist) (bestincR-1,dist) (bestincR+1,dist)三个点拟合出抛物线
            // bestincR+deltaR就是抛物线谷底的位置，相对SAD匹配出的最小值bestincR的修正量为deltaR
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            // 抛物线拟合得到的修正量不能超过一个像素，否则放弃求该特征点的深度
            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            // 通过描述子匹配得到匹配点位置为scaleduR0
            // 通过SAD匹配找到修正量bestincR
            // 通过抛物线拟合找到亚像素修正量deltaR
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            // 这里是disparity，根据它算出depth
            float disparity = (uL-bestuR);
            // 最后判断视差是否在范围内
            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
             // Depth是在这里计算的
             // depth=baseline*fx/disparity
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;   // 匹配对在右图的横坐标
                vDistIdx.push_back(pair<int,int>(bestDist,iL));  // 该特征点SAD匹配最小匹配偏差
            }
        }
    }
```



### step 4 剔除离群点：匹配距离大于平均距离2.1倍的视为误匹配

将最终成功匹配得到的N个iL iR匹配点对，进行筛选，取所有匹配点对距离的平均，剔除匹配距离大于平均距离2.1倍的离群点。

```c++
// step 4 ：剔除离群点：匹配距离大于平均距离2.1倍的视为误匹配
    // 前面SAD匹配只判断滑动窗口中是否有局部最小值，这里通过对比剔除SAD匹配偏差比较大的特征点的深度
    sort(vDistIdx.begin(),vDistIdx.end());  // 根据所有匹配对的SAD偏差进行排序, 距离由小到大
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;  // 计算自适应距离, 大于此距离的匹配对将剔除

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            //如果是单目或者没有距离的点，右目和深度都设为-1
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
```

### 源码

```c++
void Frame::ComputeStereoMatches()
{
    //将右图像中的对应特征点的水平位置和深度初始化为 -1.0
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);
    
    //计算 ORB 特征匹配距离的阈值，用于判断是否匹配成功。
    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    //获取图像的行数（高度）
    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //step 1 建立特征点搜索范围对应表vRowIndices，将iR注册到相邻行中
    //声明了一个二维向量，这个向量的长度是图像的行数，每一个向量中存储vector<size_t>类型的向量。
    //vRowIndices用来表示右图图像中每一行中的特征点对应位置索引，可以理解为第一维向量是对应行索引，第二维向量是对应列索引。
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());  //其中size_t表示无符号整数
    
    //为每一行的向量预留内存空间，这里预留了200个元素的空间
    //预留一定数量的元素空间，以避免频繁的动态内存分配和释放，从而提高性能
    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);
    
    //在右图像中遍历每个特征点 iR，然后根据特征点的位置计算其影响的行范围，并将该特征点的索引存储在 vRowIndices 中对应的行中。
    //获取右图的特征点数量
    const int Nr = mvKeysRight.size();
    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        //搜索时，对于特征点iR，以一种横带的方式再另一幅图上搜索与其匹配的点。横带的最上行是maxr，最下行是minr
        //会根据特征点iR所在金字塔层数对应的缩放系数，调整r的值，图像越大则对应的搜索范围越大
        //也就是说如果iR点的行数y=20,那么在vRowIndices[minr～maxr]上都有这个特征点索引，认为区间内都有这个特征点，用于在另一张图上搜索。
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);
        //把特征点索引iR插入vRowIndices对应行的向量中
        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    //设置搜索的深度范围 minZ、minD 和 maxD，用于排除不合适的匹配
    const float minZ = mb;  //mb=mbf/fx 
    const float minD = 0;   //最小视差, 设置为0即可
    const float maxD = mbf/minZ;   //=fx

    // step 2 ：粗匹配，对左目每个特征点iL，按照金字层级，描述子距离找出最优匹配
    // 注意：这里是校正前的mvKeys，而不是校正后的mvKeysUn
    // KeyFrame::UnprojectStereo和Frame::UnprojectStereo函数中不一致

    //vDistIdx 用于存储每个左图像特征点与其对应的最佳右图像特征点的距离和索引。
    //每个元素都是一个整数对，第一个整数表示距离，第二个整数表示特征点的索引。
    vector<pair<int, int> > vDistIdx;  //pair<int, int>是整数对格式
    vDistIdx.reserve(N);               //N是左图特征点的个数，预留了N个特征点的内存
    
    for(int iL=0; iL<N; iL++)
    {
        //对于每一个左图特征点（未矫正）找到其对应的金字塔层、x，y坐标
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;
        //对于每一个左图特征点，找到右图对应行上包含的右图特征点索引候选集
        //也就是说，上面vRowIndices已经了涵盖左图的每一行可能对应的右图特征点的索引
        //所以，对于左图的特征点，只需要查找该行vRowIndices包含的右图特征点iR索引即可，然后进行距离计算。
        const vector<size_t> &vCandidates = vRowIndices[vL];

        //vCandidates表示当前行对应的右图特征点索引
        if(vCandidates.empty())
            continue;
        
        //x的范围区间 即列的区间，要求左右图的特征点行和列都保持在一定的范围内
        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue; 

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;
        
        //当前iL的描述子
        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints

        //对候选集中的每个右相机特征点，计算其与左相机特征点的描述子距离，并找到最佳匹配。
        //遍历候选集中的匹配特征点索引
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            //判断右图特征点的金字塔层是否与左图特征点相近
            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;


            const float &uR = kpR.pt.x;
            //判断右图特征点的水平位置是否在合理范围内
            if(uR>=minU && uR<=maxU)
            {
                //当前iR的描述子
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                //计算两个描述子之间的距离
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                //遍历vCandidates中的候选匹配集，找出最优匹配，并得到最优匹配的距离和索引
                if(dist<bestDist) 
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        //step 3 细匹配：如果最优匹配在阈值内（bestDist<thOrbDist），则进行滑动窗口匹配和亚像素的抛物线拟合匹配
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            //获得该特征点所在金字塔层的图像分辨率，和特征点坐标
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            //滑动窗口匹配，根据匹配点周围5 x 5的窗口像素距离寻找精确匹配
            const int w = 5;  //// 滑动窗口的大小11*11 该窗口取自resize后的图像
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            //滑动窗口的滑动范围为（-L, L）,提前判断滑动窗口滑动过程中是否会越界
            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                // 横向滑动窗口
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                // 窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1); // 一范数，计算差的绝对值

                if(dist<bestDist)
                {
                    bestDist =  dist;   // SAD匹配目前最小匹配偏差
                    bestincR = incR;    // SAD匹配目前最佳的修正量
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            // 做抛物线拟合找谷底得到亚像素匹配deltaR
            // (bestincR,dist) (bestincR-1,dist) (bestincR+1,dist)三个点拟合出抛物线
            // bestincR+deltaR就是抛物线谷底的位置，相对SAD匹配出的最小值bestincR的修正量为deltaR
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            // 抛物线拟合得到的修正量不能超过一个像素，否则放弃求该特征点的深度
            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            // 通过描述子匹配得到匹配点位置为scaleduR0
            // 通过SAD匹配找到修正量bestincR
            // 通过抛物线拟合找到亚像素修正量deltaR
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            // 这里是disparity，根据它算出depth
            float disparity = (uL-bestuR);
            // 最后判断视差是否在范围内
            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
             // Depth是在这里计算的
             // depth=baseline*fx/disparity
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;   // 匹配对在右图的横坐标
                vDistIdx.push_back(pair<int,int>(bestDist,iL));  // 该特征点SAD匹配最小匹配偏差
            }
        }
    }
    // step 4 ：剔除SAD匹配偏差较大的匹配特征点
    // 前面SAD匹配只判断滑动窗口中是否有局部最小值，这里通过对比剔除SAD匹配偏差比较大的特征点的深度
    sort(vDistIdx.begin(),vDistIdx.end());  // 根据所有匹配对的SAD偏差进行排序, 距离由小到大
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;  // 计算自适应距离, 大于此距离的匹配对将剔除

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}
```



## 四、RGBD匹配——Frame::ComputeStereoFromRGBD

### Step 0 暗线（调用关系）

对于RGBD传感器的Frame::Frame构造函数中调用，用于计算特征点深度，虚拟右目特征点。

![image-20230830163339085](/home/hanbing/.config/Typora/typora-user-images/image-20230830163339085.png)

### Step 1 从未矫正的左图和深度图中获得特征点深度信息

### step 2 利用矫正后的左图和深度信息构建虚拟右目

### 源码

```c++
//从RGBD中获得深度信息并构建虚拟右目特征点
void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    //创建两个初始值为-1的长度为N的向量，用于初始化 右目 和 深度 信息
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            //从未矫正的左图和深度图中获得特征点深度信息，利用矫正后的左图和深度信息构建虚拟右目
            mvDepth[i] = d;
            //使用视差公式求解右目特征点
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}
```



## 五、计算图像边界——Frame::ComputeImageBounds

### step 0 暗线，调用关系

Frame::Frame初始化的step 3 时候，在传入第一张图像初始计算时调用的，用来计算图像的边界（宽高）

根据是否存在畸变系数来计算图像的四个边界坐标值：左上角、右上角、左下角和右下角。

### 源码

```c++
//用于计算图像边界范围的函数。根据是否存在畸变系数来计算图像的四个边界坐标值：左上角、右上角、左下角和右下角。
void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0) //mDistCoef是一个一行五列的矩阵，这里检查是否存在第一个元素是否不等于0，用来判断畸变系数是否存在，如果存在需要进行矫正
    {
        // 
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // 使用 cv::undistortPoints 函数对角点坐标进行畸变矫正，矫正结果保存在 mat 中。
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);
        //根据矫正后的坐标值计算图像的最小和最大边界值。
        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        //如果不存在畸变系数，就直接将图像的边界范围设置为整个图像大小
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}
 
```



## 六、矫正后的特征点分配到网格中——AssignFeaturesToGrid

### step 0 暗线，调用关系

Frame::Frame初始化的step 4 时候，最后一步将矫正后的特征点分配到网格中。

### step 1 为mGrid 分配内存单元

头文件中定义了一个名为 mGrid 的多维数组，它是一个 64x48 的网格，每个网格单元都存储一个std::vector<std::size_t>类型的向量，向量中储存了该网格内的特征点索引

### step 2 调用 PosInGrid 函数获取特征点对应的网格索引

对于每个关键点，通过调用 PosInGrid 函数判断它在某个网格单元中，并获取该网格单元的索引 (nGridPosX, nGridPosY)

### step 3 注册该单元网格中所包含的特征点索引

### 源码

```c++
//根据关键点的位置，将关键点分配到网格单元中，并将每个网格单元中的关键点索引存储在 mGrid 数据结构中
void Frame::AssignFeaturesToGrid()
{
    //step 1 头文件中定义了一个名为 mGrid 的多维数组，它是一个 64x48 的网格，每个网格单元都存储一个std::vector<std::size_t>类型的向量，向量中储存了该网格内的特征点索引
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    //step 2 对于每个关键点，通过调用 PosInGrid 函数判断它在某个网格单元中，并获取该网格单元的索引 (nGridPosX, nGridPosY)
    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
    //step 3 将该单元网格中包含的特征点索引注册上
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}
```



## 七、判断特征点在哪个网格中——Frame::PosInGrid

### step 0 暗线（调用关系）

在Frame中的（六、矫正后的特征点分配到网格中——Frame::AssignFeaturesToGrid）被调用。

该函数的作用是，判断给定关键点的位置是否在指定网格中，并且更新网格的行和列索引posX，posX。

mnMinX，mnMaxX，mnMinY，mnMaxY在（五、计算图像边界——Frame::ComputeImageBounds）中计算得到，表示图像的边界。

### 源码

```c++
//判断特征点在哪个网格中
bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    //通过关键点的坐标计算其在网格中的行和列索引
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}
```



## 八、图像特征转化为词袋向量——Fram::ComputeBoW()

