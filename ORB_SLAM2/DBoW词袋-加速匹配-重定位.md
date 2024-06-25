[TOC]



## 1.DBoW离线生成

DBoW词袋树是使用大量图像（大量室内和室外的图像）训练的，根本上是这些图像被提取FAST特征，并使用BRIEF描述子描述，把描述子作为特征，将所有的描述子放在一起。使用Kmeans++聚类形成的一颗词典树。（作者在论文中也使用过其他的描述子，因为orbSLAM2中用的是BRIEF，这里就以BRIEF举例）

所有描述子的集合作为初始的一个根点，第一次聚类产生K个类别做为第一层。之后再对这些K个类别做重复的操作，做L次，就产生了L层，最后一层的个数是K^L，这最后一层被称为叶子也被称为Words单词。

![IMG_20230919_160847](/home/hanbing/公共的/IMG_20230919_160847.jpg)

### 1.1word的值

每一个单词是一些个描述子的聚类，每一个单词在训练过程中计算并保存了他们的值和权重。

word : <value weight>

值就是每一个word包含的描述子值的平均，0.5以下的记为0，以上即为1 。

### 1.2word的权重

权重表示区分度，主要取决于构成这个word的一些个描述子的区分度和个数，是使用TF-IDF计算的，也就是TF*IDF ，TF代表词频(Term Frequency)，IDF代表逆向文件频率(Inverse Document Frequency)。

TF：举例来说如果这个word有一个描述子源于一个图像，那么说明不了什么。但如果这个word有99个都是来源于同一个图像，那这么高的词频（TF）说明这个word对于这个图像来说是一个比较重要的特征，区分度很高。

IDF：举例来说如果这个word的描述子全部是来源于一张图像，而没有其他的图像描述子，说明这个word的区分度很大。反之如果每一个图像都有一个描述子进来了这个word，那么很可能是是代表图像的同一特征，区分度可想而知就是0。

### 1.3Kmeans++聚类算法

#### kmeans

**K-Means采用的启发式方式很简单，用下面一组图就可以形象的描述，算法图如下：**

![img](https://img-blog.csdnimg.cn/20210527143816329.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L20wXzM3ODc0MTAy,size_16,color_FFFFFF,t_70)

上图a表达了初始的数据集，假设k=2。在图b中，我们随机选择了两个k类所对应的类别质心，即图中的红色质心和蓝色质心，然后分别求样本中所有点到这两个质心的距离，并标记每个样本的类别为和该样本距离最小的质心的类别，如图c所示，经过计算样本和红色质心和蓝色质心的距离，我们得到了所有样本点的第一轮迭代后的类别。此时我们对我们当前标记为红色和蓝色的点分别求其新的质心，如图4所示，新的红色质心和蓝色质心的位置已经发生了变动。图e和图f重复了我们在图c和图d的过程，即将所有点的类别标记为距离最近的质心的类别并求新的质心。最终我们得到的两个类别如图f。

#### kmeans++

k个初始化的质心的位置选择对最后的聚类结果和运行时间都有很大的影响，因此需要选择合适的k个质心。如果仅仅是完全随机的选择，有可能导致算法收敛很慢。K-Means++算法就是对K-Means随机初始化质心的方法的优化。K-Means++的对于初始化质心的优化策略也很简单，如下：

a)  从输入的数据点集合中随机选择一个点作为第一个聚类中心μ1;

b) 对于数据集中的每一个点xi，计算它与已选择的聚类中心中最近聚类中心的距离,  
$$
D(xi)=argmin||xi−μr||22r=1,2,...k selected ;
$$
c) 选择一个新的数据点作为新的聚类中心，选择的原则是：D(x)较大的点，被选取作为聚类中心的概率较大;

d) 重复b和c直到选择出k个聚类质心;

e) 利用这k个质心来作为初始化质心去运行标准的K-Means算法。



## 2.使用正向索引加速特征匹配

DBoW通过数的结构存储，利用一种正向索引的数据结构，可以减小特征匹配的计算量O(N^2)->O(N^2 / K^l)，加速特征匹配。

### 2.1正向索引

每个图像有一个FeatureVector 字典，字典的键NodeId是词袋树节点的id，范围在[0, k^L]内，L表示总层数（这里以最上层为0层）；值std::vector<unsigned int>是所有属于该节点特征编号集合。图中的fn,m表示在第n帧图像中的第m个特征点的特征点。DirectFile为所有图像正向索引的集合。 

```c++
//图像的正向索引格式
class FeatureVector: public std::map<NodeId, std::vector<unsigned int> >

//所有图像的正向索引集合
typedef std::vector<FeatureVector> DirectFile;
```

![img](https://img-blog.csdnimg.cn/2021052714322753.png)

### 2.2 SLAM在线计算图像的BoW、Feature向量和加速匹配

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230919173739022.png" alt="image-20230919173739022" style="zoom:50%;" />

#### 2.2.1 图像描述子计算正逆索引，结合SLAM的代码：Frame::ComputeBoW

```c++
//tracking中引用，计算当前帧的Bow
void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        // 将描述子mDescriptors转换为DBOW要求的输入格式
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);

        // 将一个图像的特征点的描述子转换成词袋向量mBowVec以及特征向量（正向索引）mFeatVe
        mpORBvocabulary->transform(vCurrentDesc,  //当前的描述子vector
                                   mBowVec,      //输出，记录该图像所有的单词id及其对应权重
                                   mFeatVec,     //输出，记录nodeid及对应的图像feature索引
                                   4 );          //从叶节点向前数的层数（正向索引时指定的层数＝Ｌ－４）
    }
}
```

1.首先将该图像的描述子向量转化为DBoW要求的格式vector< cv::Mat >

2.其次，将所有描述子按照与node计算汉明距离的评判依据，从离线创建好的vocabulary tree的第二层开始找自己归属的word， 用该描述子和每个节点的描述子计算汉明距离，选择汉明距离最小的作为自己所在的节点，一直遍历到叶子节点。 整个过程是这样的。紫色的线表示一个特征点从根节点到叶子节点的过程。最后记录出两个std::map容器，BoWFector和FeatureVector。用于之后的逆向和正向索引。

```c++
//BoWFector的格式
std::map <wordID , wordvalue>

//FeatureVector的格式
std::map <nodeID , std::vector<unsigned int>>
```

![img](https://img-blog.csdnimg.cn/3db7268bb57b49e1bb816515f079eb8f.png)

主要是调用transform直接指定了图像的描述子在（６－４）第二层开始找自己所在的节点，每一个描述子将返回两个向量，一个**是BowVec，表示了这个图像的某个描述子最终归属于哪一片叶子，并记录该叶子的ID和权重**，如果有多个描述子进入到了一个叶子，那么权重会累加变大，参考1.2中word的权重公式。ＢowＶec向量用来构造后面的逆向索引。另一个是**mFeatVec，记录了该这个描述子归属于第二层的那个node，并且记录了该特征点所属的图像索引和对应的特征点索引**。FeatVec就是一张图片的正向索引。 之后特征匹配的时候，直接把两个图像的正向索引放在一起，找第二层每个nodeid对应的特征点进行匹配。



#### 2.2.2利用正向索引加速匹配的过程，结合SLAM的代码matcher.SearchByBoW

​        假设两幅图像为A和B，当我们把图A和图B的特征点描述子，都转化为了树结构之后。就可以通过正向索引，找到指定节点下的图A图B共同的特征点，于是我们只需要在这些共同的特征点之间两两计算距离就行。所以我们希望指定某一层的node，让两个图像在这一层的node下找匹配点，这样的计算复杂度变为了。
$$
O(N^2)->O(N^2 / K^l)
 其中，小写的l表示指定的当前node层。
$$
​        注意到，正向索引的层数如果选择第0层（根节点），那么时间复杂度和暴力搜索一样。如果是叶节点层，则搜索范围有可能太小，错失正确的特征点匹配。作者一般选择第二层或者第三层作为父节点(L=6)。在orb_slam2中，作者指定了第二层的node，特征匹配的计算复杂度是

$$
O(N^2 / K^２)
$$

**下面结合代码来看一下**



```c++
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
                            float rot = kp.angle-F.mvKeys[bestIdxF].angle; // 特征点角度变化
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



#### 







## 3.使用逆向索引对重定位和回环检测的加速

逆向索引主要用来回环检测和重定位减少计算量的。

### 3.1逆向索引

每个叶节点（word）有一个反向索引IFRow，该反向索引里储存了该叶节点有那些图像的特征点，IFRows根据图像编号的升序排列。InvertedFile为所有叶节点反向索引的集合。

```c++
//叶节点的反向索引格式
typedef std::list<IFPair> IFRow 
   struct IFPair
  {
    // Entry id，图像编号
    EntryId entry_id;

    // Word weight in this entry，叶节点权重
    WordValue word_weight;
  }

//所有叶节点的反向索引集合
typedef std::vector<IFRow> InvertedFile;
// InvertedFile[word_id] --> inverted file of that word
```

下图描述的非常形象，对于查询的图像Q，该图像特征点转化为DBoW树结构之后，根据当前帧的图像中含有的word[101,103,105...180]，从数据库逆向索引中，索引出所有含有该word的图片，并且根据权重进行挑选候选帧，从而实现重定位或者回环检测功能。优点在于不用计算数据库中所有图像于当前帧的相似，从而加速计算。

![img](https://img-blog.csdnimg.cn/20210527143241535.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L20wXzM3ODc0MTAy,size_16,color_FFFFFF,t_70)




## Ｘ.一些自己的思考

在我理解中叶子（单词）的本质实际上是：稀疏的像素特征的描述子的语义表达。它和使用图像像素值的卷积或transformer特征提取的区别在于：

1.DBoW提取的特征是稀疏的，而CNN和tansformers是稠密的，DBoW能够成功用来加速SLAM中特征匹配和回环重定位非常依赖特征提取算法和描述子算法本身的鲁棒性，只有能够在不同场景下能够成功提取相同目标的特征并获得鲁棒的描述子，才能在DBoW树中成功找到对应的节点。

2.聚类的数据格式是256位二进制的描述子，而图像语义是像素值，描述子对目标做了像素+局部特征的描述，理论上只要描述子足够鲁棒，即便场景上（天亮天黑等）有区别，也可以对同一事物做近似的描述。