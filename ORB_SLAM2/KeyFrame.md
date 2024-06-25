|                                          |                                                              |      |
| ---------------------------------------- | ------------------------------------------------------------ | ---- |
| mnId                                     | KF标识符                                                     |      |
| KFcounter<br />map<KeyFrame*,int>        | 在UpdateConnections()函数中用于向mConnectedKeyFrameWeights传递的临时变量 |      |
| mConnectedKeyFrameWeights                | 存储着当前关键帧的共视关键帧及共视权重                       |      |
| mvpOrderedConnectedKeyFrames             | 存储与当关键帧的共视权重>15的关键帧信息，从大到小排序        |      |
| mvOrderedWeights                         | 存储与当关键帧共视权重>15的关键帧的权重大小，从大到小排序    |      |
| mvpMapPoints                             | 该关键帧跟踪的所有地图点对象                                 |      |
| mDescriptors                             | 关键帧的所有描述子                                           |      |
| **重定位时**                             |                                                              |      |
| mnRelocQuery                             | 表示该关键帧最后一次用于重定位查询时，查询帧的ID。           |      |
| mnRelocWords                             | 表示关键帧与查询帧的共享WordID个数                           |      |
| mRelocScore                              | 表示与mnRelocQuery帧的相似性分数                             |      |
|                                          |                                                              |      |
| mThDepth  <br />float                    | 表示地图点的深度阈值，取35倍的基线。所以深度超过该阈值的地图点被认为是远处的地图点，在一些工作中不被考虑，比如剔除荣誉关键帧时等 |      |
| mvDepth  <br />const std::vector< float> | 向量的索引表示某个地图点的索引，值代表该地图点的深度         |      |



## 一、更新共视关系——KeyFrame::UpdateConnections()

### step 1 构造KFcounter表示共视帧及权重

map<KeyFrame*,int> KFcounter;

基于该关键帧中的地图点，使用地图点获得观测关系GetObservations()，获得该地图点还有哪些帧可以看到。构造KFcounter字典，表示其他帧和共同看到的地图权重。

### step 2 更新之前帧的共视关系，维护之前帧的相应变量

对于权重>15的共视帧，从其他帧中使用AddConnection()更新共视关系，维护属于其他帧的mConnectedKeyFrameWeights、mvpOrderedConnectedKeyFrames和mvOrderedWeights成员变量。如果所有的帧共视地图点数量<15,那么选一个最大数目的建立共视关系，从其他帧的成员变量中更新共视关系并维护变量。

AddConnection()详见（二、 更新之前关键帧的共视关系——AddConnection()、UpdateBestCovisibles()）

### step 3 从当前帧中注册共视关系，维护当前帧变量，并建立父子关系

注册mConnectedKeyFrameWeights = KFcounter，把大于阈值的关键帧和权重的从大到小排序并分别储存在mvpOrderedConnectedKeyFrames和mvOrderedWeights中，这三行做的事情和AddConnection()这个函数结果相同。

然后寻找父帧（权重最大的那个），建立父子关系。

为什么不使用AddConnection()函数呢？是因为AddConnection()函数是针对之前帧的，等下一个关键帧出现，当前帧就会在下一个关键帧的线程中，借助AddConnection()更新和维护。

![image-20230910175407701](/home/hanbing/.config/Typora/typora-user-images/image-20230910175407701.png)

```c++

//! 更新共视图（基于该关键帧中的地图点与之前帧的共识关系，分别更新维护之前帧和当前帧的三个成员变量。）
/*
  ! 1. 基于该关键帧中的地图点，使用地图点获得观测关系GetObservations()，获得该地图点还有哪些帧可以看到。构造KFcounter字典，表示其他帧和共同看到的地图权重
  ! 2. 和权重>15的共视帧建立共视关系，并更新之前帧的共视关系，维护之前帧的相应变量
  !     如果所有的帧共视地图点数量<15,那么选一个最大数目的建立共视关系，从其他帧的成员变量中注册连接关系
  ! 3. 从当前帧中注册共视关系，并找到父帧建立父子关系：
  !    把大于阈值的关键帧和权重的从大到小排序并分别储存在mvpOrderedConnectedKeyFrames和mvOrderedWeights中，然后寻找父帧，建立父子关系
*/
void KeyFrame::UpdateConnections()
{
    //储存其他关键帧与本关键帧之间的权重，权重表示共视地图点个数
    map<KeyFrame*,int> KFcounter;
    //储存地图点对象指针的向量
    vector<MapPoint*> vpMP;

    {
        //获得该关键帧的所有地图点
        //mvpMapPoints是储存该关键帧地图点的向量
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //对此关键帧的所有地图点，检查该地图点还有哪些其他的关键帧看到了
    //! step 1 基于每个地图点的观测关系GetObservations()构造KFcounter 
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;
        
        if(pMP->isBad())
            continue;
        
        //获取该地图点的mObservations
        //mObservations[pKF]=idx; 表示这个地图点在某一帧上面对应的特征点索引
        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            //mit->first表示当前循环的字典中所对应的关键帧
            //mit->first->mnId表示使用当前循环所对应的关键帧的成员变量mnId
            //后面的mnId表示当前对象的成员变量mnId
            //mnId是在每一个KF初始化的时候累加的标识符，通常代表第几个KF，如果相同，说明两个是同一个关键帧
            if(mit->first->mnId==mnId)
                continue;
            //map<KeyFrame*,int> KFcounter;
            //表示当前帧与此帧的权重 +1
            KFcounter[mit->first]++;
        }
    }

    // 如果没有和其他帧共视
    if(KFcounter.empty())
        return;


    //! Step 2 对于权重>15的关键帧，并更新之前帧的共视关系，维护之前帧的相应变量
    //! 如果所有的帧共视地图点数量<15,那么选一个最大数目的建立共识关系
    //如果计数器大于阈值，则添加连接
    //如果没有关键帧计数器超过阈值，那么添加计数最多的那个关键帧
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;
    
    // vPairs记录与其它关键帧共视地图点数大于th的关键帧
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        //对权重大于15的其他KF建立共视关系
        if(mit->second>=th)
        {
            //权重大于15的其他KF储存在vPairs中
            vPairs.push_back(make_pair(mit->second,mit->first));
            //从其他帧的成员变量中注册连接关系
            //其他帧更新连接关系mConnectedKeyFrameWeights[pKF]=weight
            (mit->first)->AddConnection(this,mit->second);
        }
    }
    
    //对于没有大于15的权重，那么选择一个最大权重的KF，建立共视关系
    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }
    


    //! step 3 当前帧更新共视关系，并找到父帧建立父子关系
    //! >阈值的关键帧和权重的从大到小排序并分别储存在mvpOrderedConnectedKeyFrames和mvOrderedWeights中
    //! 如果当前关键帧是第一次更新共视图，除了初始帧之外，还要找父帧，建立父子关系
    //vPairs储存着与当前帧>15的关键帧及权重
    //将权重从大到小排序，把vPairs中的关键帧和权重分别储存到lKFs和LWs列表中
    sort(vPairs.begin(),vPairs.end());
    // lKFs和lWs记录着与当前帧
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        //push_front 后变成了从大到小顺序
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);


        // mspConnectedKeyFrames = spConnectedKeyFrames;
        //注册当前关键帧的共视关键帧及共视权重

        //这里相当于AddConnection(）函数做的事情，就是维护这三个表示共视关系的变量。
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
        
        //初次建立共视关系时，需要找父帧
        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}
```



## 二、 更新之前关键帧的共视关系——AddConnection()、UpdateBestCovisibles()

这两个函数的目的是，对之前的关键帧，更新（维护）表示共识帧和相应权重的三个成员变量

```c++
//添加共视帧和权重，更新mConnectedKeyFrameWeights
void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        //
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

//更新添加了共视帧之后的共视关系
//也就是更新从大到小排序的共视帧和相应权重成员变量：mvpOrderedConnectedKeyFrames和mvOrderedWeights
void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}
```

