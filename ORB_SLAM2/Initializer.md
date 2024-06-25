[TOC]



# Initializer模块之成员变量

| 成员变量                        | 意义                                                         |
| ------------------------------- | ------------------------------------------------------------ |
| cv::Mat mK                      | 内参                                                         |
| vector <cv::KeyPoint> mvKeys1   | F1关键点集合                                                 |
| vector <cv::KeyPoint> mvKeys2   | F2关键点集合                                                 |
| float mSigma                    | 冲投影误差阈值                                               |
| float mSigma2                   | 冲投影误差阈值平方                                           |
| int  mMaxIterations             | RANSAC的迭代次数                                             |
| vector<vector<size_t> >  mvSets | 二维容器，每一个行包含计算H和F矩阵的8个点                    |
| vector<Match>  mvMatches12      | 用来储存匹配点对，索引内两个整数分别为F1和F2的索引           |
| vector<bool>  mvbMatched1       | 表示当前索引的F1是否有匹配                                   |
| vector<bool> vbMatchesInliersF  | 索引对应的vMatches12索引中两个匹配点是否存在外点(不准的点)，如果有外点，则vbMatchesInliers[i]=false，后续这些点不可用 |
| vector<bool> vbMatchesInliersH  | 索引对应的vMatches12索引中两个匹配点是否存在外点(不准的点)，如果有外点，则vbMatchesInliers[i]=false，后续这些点不可用 |
| vP3D                            | 重建出的世界坐标系3D点，索引为初始帧的特征点索引             |
| vbGood                          | 三角化之后筛选的时候判断该点是否为好点，索引为初始帧的特征点索引，最后赋值给了tracking的vbTriangulated。 |
| nGood                           | CheckRT中每一个RT重建出来的3D点为好点的数量，用来在ReconstructF中筛选RT |
| vCosParallax                    | 表示每一个3D点的视差角，除了在checkRT中用来筛选3D点之外，最后得出的中间偏小的视差角还用来在ReconstructF中配合nGood点来筛选RT。 |

# Initializer模块之成员函数



## 一、构造函数——Initializer::Initializer()

### step 0 暗线

在tracking::track()中的step1 初始化的时候被调用

tracking::track() —> Tracking::MonocularInitialization() —> Initializer::Initializer()

### step 1 

其中ReferenceFrame代表当前帧，是tracking中的成员变量。

### 源码

```c++
Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
{
    mK = ReferenceFrame.mK.clone();

    mvKeys1 = ReferenceFrame.mvKeysUn;

    mSigma = sigma;
    mSigma2 = sigma*sigma;
    mMaxIterations = iterations;
}
```



## 二、初始化——Initializer::Initialize()

### step 0 暗线（调用关系）

在track线程中track()中单目初始化流程的Tracking::MonocularInitialization()方法中调用

### step 1 获取当前帧和初始帧的匹配信息存入成员变量中

从tracking做完特征匹配的成员变量中把数据，注册到Initializer类的成员变量中

### step 2 使用RANSAC随机N次抽取8对匹配特征点

迭代200次，每次生成随机数随机取出8对匹配点，得到的二维向量存储在mvSets中。该变量的第二维向量中储存的随机索引，对应mvMatches12中的索引，mvMatches12索引内是F1F2匹配对的两个特征点索引。

这些随机取出来的匹配对，用来计算基础矩阵和单应矩阵及其得分。

### step 3 计算N次中最优的F和H

使用200个匹配点对，计算基础矩阵和单应矩阵及其得分，并找到得分最高的8对点计算出来的H和F矩阵。

核心：**8点法SVD求解F H，RANSAC结合卡方检验最小化冲投影误差选出200次中的最优F H**

详见（三、Initializer::FindHomography()  和  四、Initializer::FindFundamental()）

### step 4 分解R t，选出正确的R t，得到3D化的地图点

由于在平面上基础矩阵有退化问题效果不好，而不在平面上单应矩阵效果也不行，所以通过两个矩阵共同计算，并比较得分的方式，选择得分高的进行重建。

核心：

1. F H使用SVD分解求出四种可能的R t
2. 对每个R t三角化所有内点，对每个3D点进行筛选和打分，最终得分最高的R t为4个中正确的R t。并且，如果这个正确的RT重建出来的3D点内点过少，或不满足阈值，那么将重新初始化。如果那个RT满足条件，那么此时三角化出来的3D点也都是经过RANSAC和checkRT中筛选后的内点。

详见（N、Initializer::ReconstructH()  和 N、Initializer::ReconstructF() ）



```c++
Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
{
    mK = ReferenceFrame.mK.clone();

    mvKeys1 = ReferenceFrame.mvKeysUn;

    mSigma = sigma;  //重投影误差阈值及其平方
    mSigma2 = sigma*sigma;
    mMaxIterations = iterations;
}

bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                             vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
// step 1 获取当前帧和初始帧的匹配信息（从tracking的成员函数中得来）
    // Reference Frame: 1, Current Frame: 2
    mvKeys2 = CurrentFrame.mvKeysUn;

    //清空匹配信息
    //vector<Match> mvMatches12;
    //typedef pair<int,int> Match  
    //mvMatches12是用来储存匹配点的，索引内两个整数分别为F1和F2的索引
    mvMatches12.clear();
    //预留F2中特征点的个数作为容量
    mvMatches12.reserve(mvKeys2.size());
    //vector<bool> mvbMatched1; 
    //表示当前索引的F1是否有匹配
    mvbMatched1.resize(mvKeys1.size());

    //将存储在tracking::mvIniMatches（vMatches12）内的匹配信息放入Initializer::mvMatches12和mvbMatched1中。
    for(size_t i=0, iend=vMatches12.size();i<iend; i++)
    {
        //如果有匹配，把vMatches12的匹配信息传入到mvMatches12中，并使mvbMatched1[i]=true
        //如果没有匹配
        if(vMatches12[i]>=0)
        {
            mvMatches12.push_back(make_pair(i,vMatches12[i]));
            mvbMatched1[i]=true;
        }
        else
            mvbMatched1[i]=false;
    }
    
//step 2 使用RANSAC运算需要的特征点对
    //有多少匹配的点（为什么不直接使用nmatches的值？）
    const int N = mvMatches12.size();

    //创建向量vAllIndices，向量长度==匹配点对的数量，向量索引内的值=其索引,
    //也就是说这个向量是[0,1,2,3,4,5,6,7,8,9....]
    //用来在后面生成随机数后从向量中取值，作为抽取到的随机特征点
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }
    // 生成每次RANSAC迭代的8个点对的集合
    //创建一个二维向量 mvSets  ，行数为mMaxIterations，列为8列，初始值为0。8列中的索引对应着mvMatches12的索引。
    //用于存储每次 RANSAC 迭代中所选取的最小样本集合。这些最小样本集合将用于计算基本矩阵（Fundamental Matrix）和单应矩阵（Homography），以进行后续的运动估计和三维重建。
    mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));
    DUtils::Random::SeedRandOnce(0);
    //RANSAC迭代mMaxIterations次
    for(int it=0; it<mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;
        // 选择一个最小集合
        for(size_t j=0; j<8; j++)
        {
            //在0到vAvailableIndices.size()-1的索引中随机取值
            //产生0到N-1的随机数
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            //mvSets的第it次迭代的第j个点对的索引为idx
            mvSets[it][j] = idx;
            //为了防止在一次取值中取出重复的点，把这一轮取出的点用最后一个元素替换
            //并删掉最后一个元素
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // step 3 计算mMaxIterations次中最优的基本矩阵和单应矩阵
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    //分数
    float SH, SF;
    cv::Mat H, F;
    //计算两个矩阵及其分数，求出最优的F和H矩阵   
    //使用this可以访问当前对象上的成员变量和参数
    thread threadH(&Initializer::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&Initializer::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F)); 

    // 等待两个线程完成
    threadH.join();
    threadF.join();

    // 计算分数的比率选择用F或是H重建
    float RH = SH/(SH+SF);

    // step 4 根据比率尝试从单应矩阵或基本矩阵进行重建（取决于阈值0.40-0.45）
    if(RH>0.40)
        return ReconstructH(vbMatchesInliersH,H,mK,R21,t21,vP3D,vbTriangulated,1.0,50);
    else //if(pF_HF>0.6)
        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,1.0,50);

    return false;
}

```



## 三、计算单应矩阵及分数——Initializer::FindHomography()

## 四、计算基础矩阵及分数——Initializer::FindFundamental()

#### **（归一化+8点法+重投影误差计算得分）**

计算这两个矩阵是极其相似的流程，所以我们放在一起讲，并以基础矩阵为例子。这个函数的目的是基于RANSAC的随机取值结果，用这些取值计算出一个最优的基础矩阵。

### step 0 暗线（调用关系）  

计算单应、基础矩阵和分数是在初始化的流程中（ Initializer::Initialize() ）

![image-20230904211903565](/home/hanbing/.config/Typora/typora-user-images/image-20230904211903565.png)

### step 1 将F1 F2中的特征点归一化（标准化）

标准化是计算基础矩阵之前重要的一步  ，如果不进行标准化，那么直接使用SVD计算F矩阵的时候，Ax=0 ，A中的各个元素（特征点对坐标）的数值差异过大，SVD分解有数值计算问题。因此进行标准化，对特征点集合施加变换T，让其满足以下条件：

1.原点=图像上点的重心

2.各个像点到坐标原点的均方根距离==根2（或均方距离=2）

**详见：（五、归一化Initializer::Normalize()）**

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230905095325807.png" alt="image-20230905095325807" style="zoom:25%;" />

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230905095337396.png" alt="image-20230905095337396" style="zoom: 25%;" />



### step 2 循环对每一次RANSAC迭代的点对计算H、F及其得分

#### step 2.1  8点法计算归一化特征点对应的基础矩阵

这里调用了ComputeF21()方法计算矩阵。详见（六、八点法计算基础矩阵——Initializer::ComputeF21()）

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230905095110610.png" alt="image-20230905095110610" style="zoom:25%;" />

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230905095139814.png" alt="image-20230905095139814" style="zoom:25%;" />

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230905095157953.png" alt="image-20230905095157953" style="zoom:25%;" />

#### step 2.2 逆归一化：转换成归一化前特征点对应的基础矩阵

```c++
 F21i = T2t*Fn*T1;
```

#### step 2.3 重投影误差+卡方检验，基础矩阵打分

重要的是卡方检验的假设、重投影误差的求取。

详见（七、卡方检验矩阵打分—Initializer::CheckFundamental()）

#### step 2.4 找到最好的那一次基础矩阵，注册F、分数

保存200次中分数最高的**基础矩阵F21**、分数score和**保存内外点信息的vbMatchesInliers**向量，vbMatchesInliers的索引内容意义：是该索引对应的vMatches12索引中两个匹配点是否存在外点(不准的点)，如果有外点，则vbMatchesInliers[i]=false。说明重建时不可用。

```C++
{
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
```

### 源码

```c++
void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    // Number of putative matches
    const int N = vbMatchesInliers.size();

     //step 1 将F1 F2中的特征点标准化
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    //归一化矩阵的逆 用来恢复基础矩阵 
    cv::Mat T2t = T2.t();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat F21i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;
    
    //step 2 循环对每一次RANSAC迭代的点对计算H、F及其得分
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(int j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        //step 2.1 逐个取出RANSAC每次迭代的点对，计算出归一化特征点对应的基础矩阵
        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);
        
        //step 2.2 转换成归一化前特征点对应的基础矩阵
        F21i = T2t*Fn*T1;
 
        //step 2.3 重投影误差+卡方检验 
        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);
        
        //step 2.4 找到最好的那一次基础矩阵，注册F、分数
        if(currentScore>score)
        {
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}
```



## 五、归一化Initializer::Normalize()

### 核心思想

归一化将x坐标和y坐标分别进行缩放，使得x坐标和y坐标的一阶绝对矩分别为1，一阶矩（期望）为0

满足以下条件：

1.原点=图像上点的重心

2.各个像点到坐标原点的均方根距离==根2（或均方距离=2）

### 附图

![image-20230905095325807](/home/hanbing/.config/Typora/typora-user-images/image-20230905095325807.png)

![image-20230905095337396](/home/hanbing/.config/Typora/typora-user-images/image-20230905095337396.png)

### 源码

```c++
void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    //获得关键点数量
    const int N = vKeys.size();
    //用于归一化之后的点坐标向量，长度改为n
    vNormalizedPoints.resize(N);
    
    //获取特征点平均坐标meanX meanY
    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }
    meanX = meanX/N;
    meanY = meanY/N;


    float meanDevX = 0;
    float meanDevY = 0;

    for(int i=0; i<N; i++)
    {
        //相对均值的偏差 x-mean 
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;
        // |x-mean|的求和
        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }
    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;
    
    //归一化的尺度因子
    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}

```

<img src="file:////home/hanbing/.config/QQ/nt_qq_8fd520cf964b05435dd14625633ac868/nt_data/Pic/2023-09/Ori/da6228d35d59450d4e86445cfd9a382e.jpg" alt="img" style="zoom: 33%;" />

## 六、八点法计算基础矩阵——Initializer::ComputeF21()

### 核心思想

基础矩阵的自由度=7，因为已知内参K，所以基础矩阵的自由度<8，等于本质矩阵=5 。详见14讲169页。

理论上我们使用7个点就可以计算出F，但是计算比较复杂，我们采用业界常用8点法（8个点对）计算基础矩阵，又简单又准确。用SVD求超定方程的最小二乘解。详见14讲书中的169页的附纸，上面写了。

**首先**：先将p1Fp2线性展开，然后用8点对构建矩阵A，基础矩阵用列向量x表示，于是得到一个AX=0的等式；

**然后**：使用SVD分解，将矩阵A分解为：1.左奇异矩阵U  2.代表奇异值的对角矩阵D 3.右奇异矩阵的逆Vt

**其次**：根据一系列的数学运算，可以得到，使 x 取A的最小奇异值对应的单位特征向量时候，可以将||AX||最小。此时 x = 右奇异矩阵V的最后一列。所以直接取右奇异矩阵的逆矩阵Vt最后一行，构造为3 x 3的基础矩阵。cv::Mat Fpre = vt.row(8).reshape(0, 3);

**再其次**：得到的Fpre并不是合格的基础矩阵，因为根据性质，基础矩阵的秩为2，所以我们将构造的Fpre矩阵再进行奇异值分解，将其奇异值第三位设为0，此时再通过奇异值分解逆运算还原秩为2的基础矩阵。为什么要把第三位设为0呢？是因为我们希望找到一个基础矩阵和Fpre最接近，损失最小||F-Fpre||F，所以将最小的奇异值设为0。

![image-20230904214840099](/home/hanbing/.config/Typora/typora-user-images/image-20230904214840099.png)



ps：理论上8点法选取8个点对就够了，但通常N>8，有时也会用16点等，增加点是提高鲁棒性但增大计算量，根据具体情况使用。



### 附图

![image-20230905095110610](/home/hanbing/.config/Typora/typora-user-images/image-20230905095110610.png)

![image-20230905095139814](/home/hanbing/.config/Typora/typora-user-images/image-20230905095139814.png)

![image-20230905095157953](/home/hanbing/.config/Typora/typora-user-images/image-20230905095157953.png)

### 源码

```c++
cv::Mat Initializer::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
{
    
    const int N = vP1.size();

    cv::Mat A(N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat u,w,vt;

    //将矩阵A奇异值分解
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    
    //取出右奇异矩阵的转置vt的最后一行，即为右奇异矩阵v的最后一列 作为 F ，并变为3x3的矩阵
    cv::Mat Fpre = vt.row(8).reshape(0, 3);
 
    //将Fpre奇异值分解
    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    
    //基于F的内在性质，其秩=2,所以将F的奇异值设为两个，第三个为0 
    //这里w并不是一个奇异值矩阵，而是代表奇异值的向量，这里把第三个设为0，再通过diag(w)组成奇异值矩阵。
    w.at<float>(2)=0;
    
    //使用新的奇异值矩阵运算得到真正的基本矩阵F并返回
    return  u*cv::Mat::diag(w)*vt;
}
```



## 七、卡方检验矩阵打分—Initializer::CheckFundamental()

### 核心思想

根据本质矩阵F理论公式，对于对应的p1p2点，第一幅图的级线l1 = F*p2 ，第二幅图的级线l2 = F\*p1；

根据单应矩阵H理论公式，对于对应的p1p2点，p1 = H*p2 ，p2 = H\*p1

**根据卡方检验理论，我们假设，对N个所有匹配点对:**

该基础矩阵F能够使得：p1点在F\*p2投影对应的极线l1上，p2点在F\*p1投影对应的极线l2上（点到直线距离 = 0）

该单应矩阵H能够使得：p1点在H\*p2投影点上，p2点在H\*p1投影点上。(两点间距离 = 0)

**循环计算分数总和**：

我们以一对点为单位，每次分别利用其中一个点和矩阵F（或H）计算另一个点的重投影误差，自由度为1,显著性为0.05，当计算得出的x^2 < 阈值th(3.84)时，我们认为这个假设成立，并计算分数， x^2越小分数越高。大于阈值th(3.84)时。我们就认为这个点不在极线上（或不是同一个点），不计算分数且把该点注册为外点。如此循环计算出mvMatches12中所有匹配点对的分数总和，作为该基础矩阵的分数。分数越大代表越好。

卡方检验详见ORB_SALM2的课件3.地图初始化部分。

### 附图

**点到直线距离的计算公式**（用于基础矩阵F检验点在极线上）

![image-20230905170109778](/home/hanbing/.config/Typora/typora-user-images/image-20230905170109778.png)

**两点之间距离的计算公式**（用于单应矩阵H检验是否是一个点）

向量的二范数，太简单，省略。

**卡方检验公式**

<img src="file:////home/hanbing/.config/QQ/nt_qq_8fd520cf964b05435dd14625633ac868/nt_data/Pic/2023-09/Ori/5f6bb7590b51eabd70a722384a1f7da3.jpeg" alt="img" style="zoom: 8%;" />



**卡方分布表**

![image-20230905170009459](/home/hanbing/.config/Typora/typora-user-images/image-20230905170009459.png)

### 源码

```c++
float Initializer::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
{

    const int N = mvMatches12.size();

    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);
    vbMatchesInliers.resize(N);

    float score = 0;

    //用于卡方检验的一些参数
    //1自由度、0.05显著性 卡方检验的拒绝阈值
    const float th = 3.841;
    //thScore 是内点的得分，用于计算累积得分
    //这个值实际上是2自由度、0.05显著性的卡方检验的拒绝阈值，应该是用来给两次重投影误差打分的
    const float thScore = 5.991;

    //计算协方差的倒数 invSigmaSquare，它是一个常数，用于将卡方值转换为概率
    //协方差矩阵是金字塔的误差权重矩阵，用来给计算出的重投影误差加权
    const float invSigmaSquare = 1.0/(sigma*sigma);

    //step 1 用图像上所有匹配点对逐个进行卡方检验，将最终的分数加在一起
    for(int i=0; i<N; i++)
    {  
        bool bIn = true;
        //获取当前匹配点对的关键点坐标：kp1 和 kp2
        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];
        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

    //step 1.1 计算第一个点的卡方检验值（加权重投影误差）及其得分
        // 计算在第二幅图像中的重投影误差 squareDist1，以及在第一幅图像中的重投影误差 squareDist2。
        // l2=F21x1=(a2,b2,c2)
        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;
        const float num2 = a2*u2+b2*v2+c2;
        
        //squareDist1是点到直线距离的平方，作为重投影误差
        const float squareDist1 = num2*num2/(a2*a2+b2*b2);
        //误差加权
        const float chiSquare1 = squareDist1*invSigmaSquare;

        //th是1自由度、0.05显著性 卡方检验的拒绝阈值
        //如果大于说明应拒绝这个观点（p1点在F\*p2投影对应的极线l1上，p2点在F\*p1投影对应的极线l2上是成立的）
        if(chiSquare1>th)
            bIn = false;

        //第一个点的得分    
        else
            score += thScore - chiSquare1;

    //step 1.2 计算第二个点的卡方检验值（加权重投影误差）及其得分
        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)
        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        const float num1 = a1*u1+b1*v1+c1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;
            
        //如果该点对存在外点，则注册到vbMatchesInliers[i]=false中
        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}
```

## 八、从F H中恢复Rt和重建——Initializer::ReconstructF() 、Initializer::ReconstructH

### step 1 从本质矩阵E中SVD求解四对可能的 RT 

详见DecomposeE()函数，和14讲书中的夹页

### step 2 以三角化的方式筛选出正确的那组Rt

首先使用CheckRT()函数得到每一组Rt重建的分数，选择分数最高的那个作为正确的Rt。

详见CheckRT()函数，和14讲书中的夹页

### step 3 赋值、返回

将正确的那组RT及其所重建的3D点和vbTriangulated返回。

```c++
bool Initializer:: ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                            cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // 从基础矩阵中恢复本质矩阵
    cv::Mat E21 = K.t()*F21*K;

    cv::Mat R1, R2, t;

    //! step 1 从本质矩阵E中SVD求解四对可能的 RT 
    DecomposeE(E21,R1,R2,t);  

    cv::Mat t1=t;
    cv::Mat t2=-t;

    //用4个Rt假设重建 并检查
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;

    //! step 2 以三角化的方式筛选出正确的那组Rt
    //使用了所有的点对重建，恢复出来的内点最多、得分最高的那一组Rt
    //这样相比使用一个点用来重建选出正确的Rt更鲁棒
    int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

    int nsimilar = 0;
    if(nGood1>0.7*maxGood)
        nsimilar++;
    if(nGood2>0.7*maxGood)
        nsimilar++;
    if(nGood3>0.7*maxGood)
        nsimilar++;
    if(nGood4>0.7*maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if(maxGood<nMinGood || nsimilar>1)
    {
        return false;
    }

    //! step 3 赋值、返回
    if(maxGood==nGood1)
    {
        if(parallax1>minParallax)
        {
            //重建的3D点集合
            vP3D = vP3D1;
            //vbTriangulated1就是vbGood = vector<bool>(vKeys1.size(),false);判断该索引的初始帧特征点三角化后是否为好点
            vbTriangulated = vbTriangulated1;
            //从帧1到帧2的变换矩阵
            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }

    return false;
}
```



## 九、从本质矩阵中分解RT——Initializer::DecomposeE()

使用SVD分解，从E中分解出RT，详见14讲 169页附纸

```c++
//从本质矩阵E中返回R T 
void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    //计算E的奇异值分解
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);

    //t为E的左奇异矩阵的最后一列
    u.col(2).copyTo(t);
    t=t/cv::norm(t);
    
    //构造t反对称矩阵Z中的W矩阵
    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    //根据旋转矩阵性质，使两种可能的行列式>0
    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}

```





## 十、三角化并找出正确的Rt——Initializer::CheckRT()

### step 1 获得两个相机的相机矩阵 K[I|0] K[R|T]

根据得到的R t 构建初始帧和当前帧对应世界坐标系的位姿

### step 2 逐匹配点三角化

对有效的内点三角化，在RANCSAC和卡方检验中检查到的特征匹配点对的外点不进行计算。

详见Triangulate(kp1,kp2,P1,P2,p3dC1)和14讲中附纸

### Step 3 第一关：检查三角化的三维点坐标是否合法（非无穷值）

检查重建的 p3dC1 XYZ中是否包含无穷大或非法值，如果是则标记为不好的点跳过

### Step 4 第二关：通过三维点深度值正负来检查是否合法

4个RT中那三个深度方向不符合条件的RT在这里几乎所有的点会被直接剔除。

### Step 5 第三关：计算空间点的重投影误差，如果大于阈值则舍弃

剔除初始帧或参考帧重投影误差较大的那些点

### Step 6 统计经过检验的3D点个数，记录3D点视差角 

如果运行到这里就说明当前遍历的这个特征点对靠谱，经过了重重检验，说明是一个合格的点，称之为good点 
将该特征点标记为好的点（vbGood 中对应的位置设置为 true），并将其三维坐标存储在 vP3D 中，同时增加 nGood

### Step 7 得到3D点中较中间偏小的视差角，并且转换成为角度制表示

用来在ReconstructF中配合nGood点来筛选RT。

```c++
int Initializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    //f是焦距 c是光心坐标
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    //储存每个特征点的视差余弦值
    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    //! step 1 获得两个相机的相机矩阵 K[I|0] K[R|T]
    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));
    //相机1光心为世界坐标系的原点坐标
    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;

    // 第二个相机的光心在世界坐标系（第一个相机坐标系）下的坐标
    // 因为我们已知 T21(第一个相机坐标系到第二个相机坐标系的变换矩阵)
    // 所以我们先确定第二个相机的光心在第二个相机坐标系下的坐标（0,0,0,1）
    // 之后在将这个坐标乘以 T21的逆矩阵即可。
    cv::Mat O2 = -R.t()*t;

    //nGood表示正确重建的数量
    int nGood=0;
    
    //! step 2 逐匹配点三角化
    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        //在RANCSAC和卡方检验中检查到的特征匹配点对的外点不进行计算
        //仅使用有效的匹配特征点
        if(!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;

        Triangulate(kp1,kp2,P1,P2,p3dC1);//详见14讲中附纸
        
    //! Step 3 第一关：检查三角化的三维点坐标是否合法（非无穷值）
    //剔除重建出来离谱的那些点
        //检查重建的 p3dC1 XYZ中是否包含无穷大或非法值，如果是则标记为不好的点跳过
        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first]=false;
            continue;
        }

    //! Step 4 第二关：通过三维点深度值正负来检查是否合法
    //4个RT中那三个深度方向不符合条件的RT在这里几乎所有的点会被直接剔除。
        cv::Mat normal1 = p3dC1 - O1;
        //cv::norm计算向量的二范数（模长）
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);
        
        // ||normal1|| x ||normal2|| x cos（夹角） / (dist1*dist2)
        // dist1 = ||normal1||  ；dist2=||normal2||
        // cosParallax为cos值
        float cosParallax = normal1.dot(normal2)/(dist1*dist2);

        // 检查深度值是否为正，在余弦值正常的条件下。如果不满足条件，则跳过。
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // 检查第二个相机前面的深度（仅当视差足够时，因为“无限”点可以轻松进入负深度）
        cv::Mat p3dC2 = R*p3dC1+t;

        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;
        
    //! Step 5 第三关：计算空间点在参考帧和当前帧上的重投影误差，如果大于阈值则舍弃
    //剔除重投影误差较大的那些点
        // 在第一幅图像中投影 p3dC1 并计算其在图像平面上的重投影误差 squareError1
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        //基于3D点计算投影，详见14讲99页的投影模型
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;
        
        //重投影误差就是点之间的距离的平方
        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        if(squareError1>th2)
            continue;

        //在第二幅图像中投影 p3dC2（使用 R 和 t）并计算其在图像平面上的重投影误差 squareError2。
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        if(squareError2>th2)
            continue;

        //! Step 6 统计经过检验的3D点个数，记录3D点视差角 
        // 如果运行到这里就说明当前遍历的这个特征点对靠谱，经过了重重检验，说明是一个合格的点，称之为good点 
        // 如果两个重投影误差都小于阈值 th2，则将该特征点标记为好的点（vbGood 中对应的位置设置为 true），并将其三维坐标存储在 vP3D 中，同时增加 nGood
        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;
        
        //一般来说都能满足，但是不知道为什么放在这里
        if(cosParallax<0.99998)
            vbGood[vMatches12[i].first]=true;
    }

//! Step 7 得到3D点中较中间偏小的视差角，并且转换成为角度制表示
    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end());
        size_t idx = min(50,int(vCosParallax.size()-1));
        //将这个选中的角弧度制转换为角度制
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}

```



