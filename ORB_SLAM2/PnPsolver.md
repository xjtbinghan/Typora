[TOC]

## 0.成员变量

|                      |                                                  |                                                              |
| -------------------- | ------------------------------------------------ | ------------------------------------------------------------ |
|                      |                                                  |                                                              |
| *mRansacMaxIts*      | RANSAC的迭代次数                                 | mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts)); 公式计算的值和给定输入的min |
| *mRansacMinSet*      | RANSAC求解模型至少需要的数据点个数               | 对EPnP来说是4                                                |
| *nCurrentIterations* | 当前迭代轮次                                     |                                                              |
| *nIterations*        | 传入iterate函数的给定迭代次数                    |                                                              |
| *mnInliersi*         | 某次RANSAC迭代使用EPnP求解位姿后，区分的内点个数 |                                                              |
| *mvbInliersi*        | 是否为内点的标志，Vector<bool>格式               |                                                              |
| *mRefinedTcw*        | 使用best内点数提纯得到的位姿                     |                                                              |



## 一、设置RANSAC参数 PnPsolver::SetRansacParameters()

### RANSAC原理

![image-20230924155347321](/home/hanbing/.config/Typora/typora-user-images/image-20230924155347321.png)

![image-20230924155354605](/home/hanbing/.config/Typora/typora-user-images/image-20230924155354605.png)

![image-20230924155402627](/home/hanbing/.config/Typora/typora-user-images/image-20230924155402627.png)

### 思路

设置RANSAC参数(mRansacMinInliers、迭代次数、考虑特征层数的误差阈值)

mRansacMinInliers很重要，后面EPnP能否进入提纯阶段并提纯成功都要>这个参数。

为什么mRansacMinInliers的输入值是10？

为什么最终的mRansacMinInliers是max（输入值，最小集(4)，内点数理论值(n个匹配点*内点率)）？

我的理解是：这个值是作为是否跟踪成功的兜底跟踪个数，如果跟踪成功的地图点小于这个数，那么就太少了，认为没有跟踪成功，和参考关键帧、恒速运动跟踪是一样的，成功跟踪的地图点也都要大于一定数值。如果在这里没有达到mRansacMinInliers，那么后面的g2o优化大概率不会成功到达50个点，所以如此定义。

```c++
//重定位跟踪时参数如下：pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
void PnPsolver::SetRansacParameters(double probability, int minInliers, int maxIterations, int minSet, float epsilon, float th2)
{
    mRansacProb = probability;      //0.99 P出现好模型的概率
    mRansacMinInliers = minInliers; //10   给定内点数
    mRansacMaxIts = maxIterations;  //300  给定最大迭代次数
    mRansacEpsilon = epsilon;       //0.5 RANSAC的鲁棒性参数，默认为0.5
    mRansacMinSet = minSet;         //4 计算模型所需的最小集

    N = mvP2D.size(); // 用于PnP计算的2D点数量

    mvbInliersi.resize(N);

    //!step 1 根据内点数估计值调整 内点率 max(给定内点数,最小集,内点数理论值)
    //
    int nMinInliers = N*mRansacEpsilon;  //内点数理论值
    if(nMinInliers<mRansacMinInliers)
        nMinInliers=mRansacMinInliers;
    if(nMinInliers<minSet)
        nMinInliers=minSet;
    mRansacMinInliers = nMinInliers;

    if(mRansacEpsilon<(float)mRansacMinInliers/N)
        mRansacEpsilon=(float)mRansacMinInliers/N;

    // 根据概率、epsilon和最大迭代次数设置RANSAC迭代次数。
    int nIterations;
    
    //! step 2 确定迭代次数 min(给定最大迭代次数,计算迭代次数)
    //最小内点数==N时，一次就能计算出最好的模型
    if(mRansacMinInliers==N)
        nIterations=1;
    //否则（正常情况下）根据公式计算迭代次数
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(mRansacEpsilon,3)));

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    //! step 3 根据特征点层数确定误差阈值(层数越高，误差阈值越大)
    mvMaxError.resize(mvSigma2.size());
    for(size_t i=0; i<mvSigma2.size(); i++)
        mvMaxError[i] = mvSigma2[i]*th2;
}

```



## 二、RANSAC+EPnP  PnPsolver::iterate()

### 思路

在RANSAC中，对每次迭代EPnP区分的内点数大于mRansacMinInliers时做两件事。

1. 与内点数最多的那次比较，更新mnBestInliers和BestTcw位姿

2. 尝试refine，使用mnBestInliers提纯再次EPnP计算位姿，如果refine成功了，那么不进行ransac之后的轮次，直接返回提纯的内点索引和位姿Tcw。

如果迭代次数用尽了，还是没有一次能够refine成功，那么就返回记录的Best内点和位姿。bNoMore设置为Ture	





![ORB-SLAM2系列之详解PnPsolver类](https://pica.zhimg.com/70/v2-813161ab3a25859533b9d7f1272caa27_1440w.image?source=172ae18b&biz_tag=Post)



