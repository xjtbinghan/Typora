[TOC]



# ORBmatcher模块之成员变量

| 成员变量 | 意义 | 来源&值 |
| -------- | ---- | ------- |
|          |      |         |
|          |      |         |
|          |      |         |
|          |      |         |
|          |      |         |
|          |      |         |
|          |      |         |

# ORBmatcher模块之成员函数

## 一、构造函数——Initializer::Initializer()

### step 0 暗线

在tracking::track()中的step1 初始化的时候被调用

tracking::track() —> Tracking::MonocularInitialization() —> ORBmatcher matcher(0.9,true)

```c++
 //初始化ORBmatcher对象，用于匹配
        ORBmatcher matcher(0.9,true);
        /**
         * mvbPrevMatched为前一帧的特征点的坐标，存储了mInitialFrame中哪些点将进行接下来的匹配
         * mvIniMatches用于存储mInitialFrame,mCurrentFrame之间匹配的特征点
        */
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);
```

### step 1 

其中ReferenceFrame代表当前帧，是tracking中的成员变量，

### 源码

```c++
ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}
```





## 二、单目初始化时的特征匹配——ORBmatcher::SearchForInitialization()

用于单目初始化中用于参考帧和当前帧的特征点匹配

### 流程图

这里代表的是Tracking::MonocularInitialization()   tracking中单目初始化的step 2 。这个算法主要完成单目初始化的特征匹配环节

![image-20230904203559973](/home/hanbing/.config/Typora/typora-user-images/image-20230904203559973.png)

### step 0 暗线

在tracking::track()中的step1 初始化的时候被调用

tracking::track() —> Tracking::MonocularInitialization() —>matcher.SearchForInitialization（）

```c++
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);
//mInitialFrame：初始帧对象
//mCurrentFrame：当前帧对象
//mvbPrevMatched：长度==F1中的特征点个数，索引代表F1的特征点索引，值代表与该F1特征点成功匹配的F2特征点的坐标pt。
//mvIniMatches：长度==F1中的特征点个数，索引代表F1的特征点索引，值代表与该F1特征点成功匹配的F2特征点的索引。在该方法中，mvIniMatches参数赋值到成员变量vnMatches12中。

```

### step 1 窗口搜索候选匹配点

​		首先获得初始帧F1的特征点总数，建立for循环，取出每一个特征点的坐标pt和金字塔层。只考虑金字塔层为0也就是原始分辨率图像层上的特征点，对于这些特征点，按照其坐标pt在当前帧F2的图像相同位置，在大小为100x100的窗口内搜索F2中的特征点，如果搜索到了，则作为候选匹配点。其中主要调用了Frame类中的GetFeaturesInArea()方法，用来获得窗口内的F2特征点。这里作者考虑到两帧之间位移比较小，使用100的窗口能够筛选掉绝大部分的无效点，用于加速。

<img src="https://img-blog.csdnimg.cn/692d14596c3e405e9fa26dc45d15cea5.png" alt="img"  />

```c++
    //step 1 对每个F1特征点，按照其坐标在窗口中搜索F2特征点作为候选匹配
    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;

        //只使用原始分辨率金字塔上的特征点
        if(level1>0)
            continue;

        //基于每一个F1特征点vbPrevMatched[i1]坐标，在F2的半径窗口内搜索所有的候选匹配特征点
        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

        if(vIndices2.empty())
            continue;
```

### step 2 候选匹配点按照描述子距离找出最优次优匹配

​		对于vIndices2储存的N个在窗口内的候选特征点，按照描述子距离，找出最优匹配和次优匹配。其中每帧的特征点信息在构造帧的时候就注册在了Frame类中，因此，调用Frame类中的mDescriptors成员变量可以获得该帧某个特征点的描述子。使用ORBmatcher::DescriptorDistance按照一范数计算描述子距离。

```c++
      //step 2 在窗口内的F2候选匹配点中，找到最优和次优匹配
        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;
            
            //取出窗口内F2特征点的描述子
            cv::Mat d2 = F2.mDescriptors.row(i2);
            //计算特征点描述子距离
            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }

            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }
        
```

### step 3 最优点筛选

​		对于找到的最优点，要求距离小于阈值、满足最优点和次优点的比例设置，如果该F2特征点已经被之前的F1特征点匹配上了，那么将之前的匹配删除，替换为该F2特征点作为当前点去匹配。匹配的点在vnMatches12和vnMatches21中双向注册。

```c++
//step 3 筛选最优次优匹配
        //3.1 距离小于阈值
        if(bestDist<=TH_LOW)
        {
            //3.2 满足最优/次优的比例
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                //3.3 如果当前F2的特征点已经和之前的F1点做匹配了，说明发生了重复匹配
                //将原来的匹配删掉替换为当前的
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                //3.4 满足条件后双向匹配并注册距离
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;
```

### step 4 计算F1F2特征点所有匹配的主方向差，构建直方图

​		按照ORB特征点提取算法，每一个特征点都有其特征主方向，计算第三步中匹配好的F1和F2特征点的特征主方向的角度差，将差值分数bin作为横坐标，F1特征点索引作为纵坐标，构建 vector<int> rotHist 作为角度差直方图。这样，我们就得到了所有的匹配点对之间的特征主方向的差值统计，类似于如下图。

举一个例子吧：

    如果计算角度差值为5° 则5/360 * 30 = 0.42 放入第0个直方图内。
    如果计算角度差值为15° 则15/360 * 30 = 1.26 放入第1个直方图内。
    如果计算角度差值为140° 则140/360 * 30 = 11.67 放入第11个直方图内。
![img](https://img-blog.csdnimg.cn/46efc2604b5347f1bced92f0edf496ab.png)

！！！计算的过程中，作者的源码有bug，正确的是factor = HISTO_LENGTH/360.0f，然而在代码中作者写成了factor = 1.0f/HISTO_LENGTH; 是错误的。

```c++
// step 4 计算匹配点旋转角度差，构建角度差直方图
                if(mbCheckOrientation)
                {
                    float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;

                    //factor = HISTO_LENGTH/360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
```

### step 5 剔除非主流方向的匹配点

​		考虑到初始帧和当前帧之间的相对位移，所以当前帧的图像应该相较于初始帧的图像额外有一个统一的移动特征，这一点反映到图像上，就是特征主方向角度差的一致性。因此，对于我们已经得到的所有匹配对，需要进行最后一步剔除。

​		首先使用ORBmatcher::ComputeThreeMaxima()方法，筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引。之后，我们利用rotHist向量可以找到所有不在这三个bin之中的F1特征点索引，再将vnMatches12[idx1]的值从F2特征点的索引设置为-1,即可完成剔除。最终得到的所有匹配点对存储在vnMatches12中并返回给Tracking类的成员变量mvIniMatches，最终所有的匹配数量存储在nmatches中并返回。

```c++
//step 5 筛除旋角度差直方图中“非主流”部分
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;
        
        // 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);
        
        // 剔除掉不在前三的匹配对，因为他们不符合“主流旋转方向”  
        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }

    }
```



## 三、参考关键帧跟踪时——ORBmatcher::SearchByBoW

### step 1 创建一些后面匹配需要的变量

vpMapPointsKF：参考关键帧的地图点向量，匹配成功后用来给对应的特征点注册；

vpMapPointMatches：用于储存成功匹配的之后的当前帧特征点 对应 地图点的向量 ；

nmatches: 计算匹配点对的总数；

rotHist： 用来统计特征方向差直方图的容器。

### step 2 while循环，将ID相同的node点对应上

获得两个图的正向索引，假设参考关键帧的特征点分入了123579node,当前帧的特征点分入了23789node，使用while(KFit != KFend && Fit != Fend)结构加上三个if判断，将正向索引相同ID的node对应上

if：nodeid相同，那么直接进入step3匹配。匹配后f1it++ f2it++

if：nodeid1(f1it->first)  < nodeid2(f2it->first)，使f1it移动到>=f2it的最小值，再进入循环判定；

if:   nodeid1(f1it->first)  > nodeid2(f2it->first)，使f2it移动到>=f1it的最小值，再进入循环判定；

但这样有BUG，如果最后一个nodeid1 == nodeid2 会因为 不满足while的条件而直接终止循环，没办法做最后一个node的匹配。

<img src="file:////home/hanbing/.config/QQ/nt_qq_8fd520cf964b05435dd14625633ac868/nt_data/Pic/2023-09/Ori/d7e438bca3e9d4f6a108b73ac050e8ca.jpeg" alt="img"  />

#### BUG

有BUG，如果两个帧的最后一个nodeID相同的的时候，会导致while的条件为flase，这时候还没有进入到匹配环节，就退出了。遗漏了最后一个node的匹配

![image-20230920144807713](/home/hanbing/.config/Typora/typora-user-images/image-20230920144807713.png)

### step 3 同一node的特征点两两匹配，找出最优匹配，计算匹配点对特征方向差直方图

取出KF1中所有重建了地图点并且地图点不为bad的特征点，用来和F2帧匹配。重建了地图点说明该特征点在之前被认为是良好匹配。这些特征点计算描述子的距离，满足阈值和与次优匹配的比例条件后找出最优匹配。最优匹配将有资格注册地图点，并计算特征方向差，第四步的非主流直方图将是他们最后的关卡，如果不通过，依然会被淘汰并收回地图点。

### step 4 剔除非主流方向的匹配点对

在step3中每一个符合条件的最优匹配，都会计算两个特征点方向差，对于所有的匹配点对，形成了一个直方图。剔除直方图中除了三大主方向之外的其他匹配点对。同时剔除他们的地图点。

```c++
//普通帧与参考关键帧之间的跟踪
/*
 * @brief 通过词袋，对关键帧的特征点进行跟踪
 * 步骤
 * Step 1：分别取出属于同一node的ORB特征点(只有属于同一node，才有可能是匹配点)
 * Step 2：遍历KF中属于该node的特征点
 * Step 3：遍历F中属于该node的特征点，寻找最佳匹配点
 * Step 4：根据阈值 和 角度投票剔除误匹配
 * Step 5：根据方向剔除误匹配的点
 * @param  pKF               参考关键帧(F1)
 * @param  F                 当前普通帧(F2)
 * @param  vpMapPointMatches F中地图点对应的匹配，NULL表示未匹配
 * @return                   成功匹配的数量
 */
//! 有BUG，如果两个帧的最后一个nodeID相同的的时候，会导致while的条件为flase，这时候还没有进入到匹配环节，就退出了。遗漏了最后一个node的匹配
int ORBmatcher::SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
{
    
    //! step 1 创建一些后面匹配需要的变量
    //取出参考帧的地图点对象向量
    const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();

    //包含每个匹配成功的当前帧F2特征点索引和对应的地图点,NULL表示未匹配.
    vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));
    
    //取出参考帧的正向索引
    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

    int nmatches=0;

    //特征点角度旋转差统计用的直方图
    //用于匹配完成后，最后剔除前三个主流直方图之外的非主流匹配点对
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    // 将0~360的数转换到0~HISTO_LENGTH的系数
    // 原作者代码是 const float factor = 1.0f/HISTO_LENGTH; 是错误的，更改为下面代码  
    const float factor = HISTO_LENGTH/360.0f;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    //取出参考帧和当前帧的BoW正向索引开头和结尾，用于while做循环
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

    //! step 2 为了使所有nodeid相同的特征点进行匹配，用了一个循环和三个条件语句，终止条件是两个nodeid都到了最后一个
    while(KFit != KFend && Fit != Fend)
    {
        //! step 2.1 如果nodeID正好相同时，进入step3遍历F1和F2的特征点两两计算距离匹配
        // 后面的两个elseif 是基于两帧nodeid的大小，调整使其相同，调整的方式是让id小的往前移动，这样就不会错过*
        if(KFit->first == Fit->first)
        {
            //取出该nodeID下F1和F2的特征点集合
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF = Fit->second;

            //! step 3 在该nodeID下，F1的每一个特征点，与F2的所有特征点计算距离，得到最佳匹配
            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            {
                //取出该nodeID下,每一个F1特征点的地图点和描述子
                const unsigned int realIdxKF = vIndicesKF[iKF];
                MapPoint* pMP = vpMapPointsKF[realIdxKF];
                const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);
                
                //如果该地图点是空，说明这个这个地图点对应的特征点是之前剔除的外点。
                //如果是bad，说明这个地图点是之前三角化重建时重投影误差过大的外点。因此，这些特征点不用来匹配
                if(!pMP)
                    continue;
                if(pMP->isBad())
                    continue;                

                //用于存储最佳匹配，次佳匹配和最佳匹配特征点ID（F2的ID）
                int bestDist1=256;
                int bestIdxF =-1 ;
                int bestDist2=256;

                //! step 3.1 遍历该nodeID下,F2的所有特征点计算距离,
                for(size_t iF=0; iF<vIndicesF.size(); iF++)
                {
                    //取出每一个F2特征点和描述子
                    const unsigned int realIdxF = vIndicesF[iF];

                    //如果F2的特征点已经继承了F1的地图点了，说明已经和F1的对应特征点完成了匹配，直接找F2的下一个特征点 
                    if(vpMapPointMatches[realIdxF])
                        continue;

                    const cv::Mat &dF = F.mDescriptors.row(realIdxF);

                    //计算描述子距离
                    const int dist =  DescriptorDistance(dKF,dF);

                    //构造最佳匹配，次佳匹配和最佳匹配特征点ID
                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdxF=realIdxF;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }
                //! step 3.2 最优匹配的筛选，距离小于阈值,且小于次优的0.7
                if(bestDist1<=TH_LOW)
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))

                    //! step 3.3通过筛选后，注册匹配特征点的地图点
                    {
                        vpMapPointMatches[bestIdxF]=pMP;

                        //这里的realIdxKF是当前遍历到的关键帧的特征点id
                        const cv::KeyPoint &kp = pKF->mvKeysUn[realIdxKF];
                        
                        //! step 3.4计算参考关键帧和当前帧两个特征点之间的特征主方向偏差所在的直方图
                        if(mbCheckOrientation)
                        {
                            // angle：每个特征点在提取描述子时的旋转主方向角度，如果图像旋转了，这个角度将发生改变
                            // 所有的特征点的角度变化应该是一致的，通过直方图统计得到最准确的角度变化值
                            float rot = kp.angle-F.mvKeys[bestIdxF].angle; // 该特征点的角度变化值
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor); //将rot分配到bin组, 四舍五入, 其实就是离散到对应的直方图组中
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxF);  // 直方图统计
                        }
                        nmatches++;
                    }
                }

            }

            KFit++;
            Fit++;
        }
        //! step 2.2 第一帧nodeID小时，前移f1it
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        //! step 2.3 第二帧nodeID小时，前移f2it
        else
        {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }

    //! step 4 剔除三大主流方向之外的非主流特征方向差
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMapPointMatches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}
```



## 四、恒速运动模型跟踪时：matcher.SearchByProjection()

**恒速模型跟踪时使用的特征匹配方法，关键点就是：**

1. 使用地图点投影得到的uv 和地图点对应LastFrame的特征点金字塔尺度系数寻找FeatureArea范围内的所有特征点，范围内的都是候选匹配点。使用这个函数**CurrentFrame.GetFeaturesInArea()**找FeatureArea范围内的所有特征点。

2. 匹配时并不是上一帧的特征点描述子，而是投影的地图点代表描述子和当前帧计算距离。

**单目和 双目&RGBD在这个函数中做的事情还不一样**

​       他们还根据当前帧和上一帧的Z方向平移，确定了是正向或反向投影，并根据这个关系，使用CurrentFrame.GetFeaturesInArea()函数找FeatureArea范围内的所有特征点。

![image-20230922110904574](/home/hanbing/.config/Typora/typora-user-images/image-20230922110904574.png)

![image-20230922110930497](/home/hanbing/.config/Typora/typora-user-images/image-20230922110930497.png)

```c++
//*在Tracking::TrackWithMotionModel()函数中调用,地图点投影寻找匹配
//使用地图点的投影到当前帧上，根据地图点对应LastFrame的特征点octive找对应FeatureArea范围内，将该范围内的当前帧特征点作为候选点。
//计算与这些候选点之间的距离，找到最优匹配并经过特征方向直方图的筛选，通过后注册当前帧特征点和地图点的连接。
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
{
    int nmatches = 0;

    // 构建特征主方向直方图容器
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
    rotHist[i].reserve(500);
    const float factor = HISTO_LENGTH/360.0f;

    //取出当前帧位姿和上一帧位姿
    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    
    const cv::Mat twc = -Rcw.t()*tcw; //世界坐标系相对当前帧的平移向量twc

    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);
    
    //计算上一帧到这一帧的位移t
    const cv::Mat tlc = Rlw*twc+tlw;  
    //(双目和RGBD)根据相对平移向量的Z分量和bMono参数的值，确定了是否进行正向（bForward）或反向（bBackward）投影匹配。
    const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;
    const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;

    //! step 1 找上一帧观测到的地图点，投影到当前帧，按照金字塔层找FeatureArea，在区域内的候选点中找最优匹配,
    for(int i=0; i<LastFrame.N; i++)
    {
        MapPoint* pMP = LastFrame.mvpMapPoints[i];

        if(pMP)
        {
            if(!LastFrame.mvbOutlier[i])
            {
                //! step 1.1地图点投影到当前帧
                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw*x3Dw+tcw;

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);

                if(invzc<0)
                    continue;
                //投影到当前帧的uv
                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;
                
                //获得当前地图点在上一帧的对应特征点金字塔层数
                int nLastOctave = LastFrame.mvKeys[i].octave;

                // Search in a window. Size depends on scale
                // 根据金字塔层数获取对应的尺度系数
                float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

                vector<size_t> vIndices2;

                if(bForward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave);
                else if(bBackward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, 0, nLastOctave);
                else
                    //! step 1.2按照金字塔层找FeatureArea
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave-1, nLastOctave+1);

                if(vIndices2.empty())
                    continue;
                
                //地图点的代表描述子
                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;
               
                //! step 1.3 使用地图点的代表描述子与区域内的所有特征点匹配
                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)
                            continue;

                    if(CurrentFrame.mvuRight[i2]>0)
                    {
                        const float ur = u - CurrentFrame.mbf*invzc;
                        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                        if(er>radius)
                            continue;
                    }

                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }
                
                ////! step 1.3 符合条件的最优匹配，注册地图点
                if(bestDist<=TH_HIGH)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = LastFrame.mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
            }
        }
    }

    //! step 2 计算直方图，剔除非主流方向特征点，剥夺其地图点
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}
```

## 五、重定位时的投影

## 六、跟踪局部地图时的投影

## 七、局部建图中的投影

### 7.1新建地图点投影

### 7.2地图点融合的投影
