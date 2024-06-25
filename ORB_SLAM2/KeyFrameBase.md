## 在重定位跟踪时，借助DBow逆向索引和共视关系找候选帧DetectRelocalizationCandidates(Frame *F)

### 思路

核心就是四个成员变量（数目、分数、共视组分数、重定位跟踪候选）的维护：

1.首先通过DBow的逆向索引找到有共享WordID的关键帧，维护 lKFsSharingWords；

2.从 lKFsSharingWords中筛选出共享Word>阈值的帧，通过DBow计算与每帧的相似分数，维护lScoreAndMatch；

3.从 lScoreAndMatch的每一个帧中找到共视关系权重前10的十个帧，计算这11个帧对查询帧的总相似性得分，并找到11个帧中得分最高的那帧，维护lAccScoreAndMatch；找到所有组的最高累积分bestAccScore；

4.取最高累积分的0.75作为阈值，累计分大于阈值的那些组中，取该组得分最高的那个帧存入vpRelocCandidates中，作为重定位跟踪的候选帧集合。

**关键变量**

list<KeyFrame*>  lKFsSharingWords：保存共享WordID的关键帧，每个关键帧有mnRelocQuery,mnRelocWords成员变量。

list<pair<float,KeyFrame*> >lScoreAndMatch：储存相似帧及其分数的列表<相似分数 ， 相似帧 >。

list<pair<float,KeyFrame*> > lAccScoreAndMatch：< 累积的得分,这11个帧中相似性最高的帧>

vector<KeyFrame*> vpRelocCandidates ： 重定位跟踪的候选帧集合

### 源码

```c++
vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    //储存共享关键帧的列表,每个关键帧有mnRelocQuery,mnRelocWords和mRelocScore成员变量
    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        //! step 1 循环当前帧的每一个WordID,找到所有与当前帧共享wordID的关键帧，并累计他们的共享数量,储存在lKFsSharingWords中
        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            //逆向索引结构mvInvertedFile  :  std::vector<list<KeyFrame*> >
            //! 1.1 使用逆向索引结构找到该帧的每一个wordID对应的其他关键帧 
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];
            //!1.2 遍历对应的所有其他帧，使mnRelocQuery为当前帧的ID，并更新mnRelocWords数目,储存在lKFsSharingWords中
            //mnRelocQuery是关键帧的成员变量，其值为当前帧的mnID
            //mnRelocWords表示关键帧与当前帧的共同WordID个数
            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnRelocQuery!=F->mnId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();
    
    //! step 2 计算所有相似帧的相似分数得到lScoreAndMatch，为了加速，仅与共享数目大于阈值的帧计算分数
    //! 2.1 找到最大共享数目,储存在maxCommonWords中,使最小共享数目(阈值)为最大共享数的0.8
    int maxCommonWords=0;

    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    //lScoreAndMatch储存相似帧及其分数的列表<相似分数 ， 相似帧 >
    list<pair<float,KeyFrame*> > lScoreAndMatch;

    int nscores=0;

    //!2.2 调用DBow的score函数,计算大于数目阈值的相似帧的相似分数
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        
        // 仅与共享单词数大于阈值的关键帧进行比较
        
        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            // 调用DBow的score函数，计算两帧bowvec中所有相同wordID的分数并累加，每一个相同的wordID分数==两帧在该word上的权重相乘
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            //在该帧上注册分数mRelocScore
            pKFi->mRelocScore=si;
            //lScoreAndMatch储存相似帧及其分数的列表<相似分数 ， 相似帧 >
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();
    
    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    //! step 3 对于第二步中筛选出来的pKFi，每一个都要抽取出自身的共视（共享地图点最多的前10帧）关键帧分为一组，
    //! 计算该组整体相似性得分，记为AccScore。找出整体得分最高的那个，记为bestAccScore.
    //! 所有组得分大于0.75*bestAccScore的，均当作闭环候选帧。
    //通过DBow仅仅是对图像的特征点聚类，试想一下，如果有一个关键帧使车辆在行使的过程中在很远的位置得到的，
    //并且与我们查询帧非常的相似，那么很显然我们不能使用这一关键帧用来重定位。所以我们不光要使用2D的特征点DBow来记分，还要使用共视图的3D关系来区分。
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        
        KeyFrame* pKFi = it->second;
        //返回共视图中与此keyframe连接的权值前10的节点keyframe
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        //accScore:11个帧的累加得分
        //bestScore:11个帧中的最大相似性得分
        float bestScore = it->first;
        float accScore = bestScore; //先把第二部筛出来的那个关键帧的分数加上
        KeyFrame* pBestKF = pKFi;

        //计算10个共视帧的累加得分
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            // 说明pKF2与F没有共同的单词，进行下一个关键帧
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;

            accScore+=pKF2->mRelocScore;

            //得到11个帧中的最大相似性得分bestScore
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        //lAccScoreAndMatch :<累积的得分,这11个帧中相似性最高的帧>
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));

        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    //!step 4 取最高累积分的0.75作为阈值，累计分大于阈值的那些组中，取该组得分最高的那个帧存入vpRelocCandidates中，作为重定位跟踪的候选帧集合。
    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}
```

