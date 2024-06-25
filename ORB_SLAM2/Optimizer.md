[TOC]



## （初始化过程）全局BA—— Optimizer::GlobalBundleAdjustemnt

这个代码的关键就是把全局地图map的关键帧和地图点放进去了，所以调用了BA之后就做了全局的BA优化，如果不想做全局，只要改称局部地图就行了，核心在于BA的流程。

```c++

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    //vpKFs：关键帧向量
    //vpMP：地图点向量 
    //nIterations：优化迭代次数
    //pbStopFlag：用于控制优化过程是否停止的标志。如果 pbStopFlag 指向的布尔值为 true，则优化过程可能会提前终止。
    //nLoopKF：表示循环关键帧的数量。在一些 SLAM 系统中，为了提高鲁棒性和一致性，可能会选择特定的关键帧作为循环关键帧，以便在全局图优化中考虑它们。
    //bRobust：用于指示是否使用鲁棒的优化方法。鲁棒的优化方法通常会考虑误差分布的不确定性，并尝试减小离群值的影响
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}

```



### BA优化地图点和位姿——Optimizer::BundleAdjustment()

​		同时优化地图点VertexSBAPointXYZ和位姿VertexSE3Expmap两种顶点的二元边g2o优化框架。思路是通过地图点的GetObservations()方法获得所连接的关键帧，基于此构建二元边计算误差。 配合BA_G-N_g2o食用呦 ～～

```c++

void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());
    
    //! 构建图模型
    g2o::SparseOptimizer optimizer;
    //实例化Block、Linearsolver和OptimizationAlgorithm
    //BlockSolver_6_3 == g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
   
    optimizer.setAlgorithm(solver);
    
    //强制停止
    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    //! 逐个设置关键帧作为顶点，并按照关键帧的mnID设置顶点ID
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        //g2o::VertexSE3Expmap 表示一个使用李代数表示的SE(3)位姿顶点
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        //将每个顶点的位姿设为李代数形式，并设置为顶点的初始估计值
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        //设置顶点的ID为关键帧的mnID
        vSE3->setId(pKF->mnId);
        //把第一个顶点设置为固定，该变量不优化（即把初始帧的位姿[I 0]设置为固定，防止世界坐标系的变换）
        vSE3->setFixed(pKF->mnId==0);
        //添加顶点
        optimizer.addVertex(vSE3);
        //设置maxKFid为最大的那个关键帧mnID
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    //! 逐个设置地图点作为顶点
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        //g2o::VertexSBAPointXYZ表示 3D地图点类型的顶点
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        //设置初始值
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        //延续顶点的ID，+1是因为考虑到第一个点是0防止重合
        const int id = pMP->mnId+maxKFid+1; 
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        //添加顶点
        optimizer.addVertex(vPoint);
        
        //! 基于每个地图点的观测关系设置边
       const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        //const_iterator 类型被明确指定为 map<KeyFrame*, size_t> 的迭代器类型，该类只能读取容器中的元素，而不能修改它们
        //mit是一个迭代器对象，表示当前迭代的位置
        //mit!=observations.end()作为继续下一轮的条件，当为ture的时候还可以继续下一轮遍历，如果已经到最后一个了，那么就不进行下一轮遍历
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            //observations = mObservations ：mObservations[pKF]=idx
            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            nEdges++;
            //kpUn作为该帧与该3D点对应的观测点
            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            //如果是单目，说明是在单目初始化的过程中调用的
            if(pKF->mvuRight[mit->second]<0)
            { 
                //观测值
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;
                
                //EdgeSE3ProjectXYZ用于表示SLAM中相机投影地图点的约束，误差维度，误差类型，和连接的顶点类型已经写好了。误差和雅可比矩阵的计算也已经在g2o框架中写好了，所以不用
                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
                //先添加地图点顶点 再添加位姿顶点
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                //设置观测值
                e->setMeasurement(obs);
                //设置信息矩阵（根特征点所在金字塔层数相关）
                //mvInvLevelSigma2：各层级尺度系数平方的倒数{1，0.694 ，0.482 ，0.355······}
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                //信息矩阵  [invSigma2  0 ]
                //        [ 0  invSigma2]
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
                
                //鲁棒就是核函数使用huber而不是二范数，并且给Huber函数的得尔塔设为thHuber2D
                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }
                //边对象中添加内参，为了计算误差
                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                //在图里添加边
                optimizer.addEdge(e);
            }
            else
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);
            }
        }
        
        //如果这个地图点没有找到对应的关键帧，那就把这个地图点从顶点中剔除
        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    //! 初始化优化器并Optimize
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    //! 更新优化的变量

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }


```



## （跟踪线程）当前帧位姿优化—Optimizer::PoseOptimization(Frame)

### 思路

仅仅优化位姿而不优化3D点，把该帧的pose作为唯一的顶点，2D点作为观测值，2D-3D重投影误差使用Huber核函数作为损失。优化结束后根据投影误差和卡方阈值的比较区分内点和外点。

SLAM只优化位姿在g2o框架中也是有定义好的顶点和边类型，直接使用即可。

### step 1 构建g2o图模型

Blocksolver、Linearsolver、OptimizationAlgor、SparseOptimizer图模型的创建。

### step 2 添加顶点

顶点类型使用g2o::VertexSE3Expmap，设置初始值并转化为李群格式

### step 3 添加边

边的类型使用 g2o::EdgeSE3ProjectXYZOnlyPose，这是SLAM仅优化位姿的情况加g2o定义好的。只需要在边中设置观测值，信息矩阵、相机内参fx、fy、cx、cy，核函数和地图点即可。

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230921134640979.png" alt="image-20230921134640979" style="zoom:67%;" />

### step 4 开始优化，区分内外点

总共优化四次，每次10itr,优化后，将观测分为outlier和inlier，outlier不参与下次优化。区分外点是每一次的10轮优化后，计算所有点的卡方误差（这个卡方误差是g2o中写好的，应该是有重投影误差构造的），如果卡方误差>卡方阈值，那么被判为外点，该点不进入下一轮的优化。但每一轮都会对所有点取分内外点，如果之后误差小于阈值，那么还会重新恢复到内点中。

### step 5 取出优化后的位姿注册到该帧中,返回去除外点的有效匹配个数

把优化后的位姿从李群格式转化为cv::Mat格式，并赋予当前帧。

```c++
//*当前帧位姿优化 (一个顶点 多个一元边)
int Optimizer::PoseOptimization(Frame *pFrame)
{
    //! step 1 构建g2o图模型
    g2o::SparseOptimizer optimizer;

    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);


    int nInitialCorrespondences=0;

    //! step 2 设置顶点
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    //设置初始值，转化为SE3Quat格式
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    //设置顶点ID
    vSE3->setId(0);
    //setfixed是设置固定的顶点，在全局BA中使用了把初始帧的单位位姿设为了固定，这里没有设置
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    //! step 2 设置边
    //! step 2.1 构造之后储存每个边和对应特征点(地图点)索引的向量容器
    const int N = pFrame->N;
    
    //vpEdgesStereo储存每一个边对象
    //vnIndexEdgeStereo储存每一个边对象对应的特征点索引
    //单目
    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);    //分配内存，不是改向量的尺寸
    vnIndexEdgeMono.reserve(N);

    //立体视觉
    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);
    
    //定义了两个阈值 deltaMono 和 deltaStereo，它们用于鲁棒核Huber函数的参数设置
    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);


    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    //!step 2.2 循环添加边
    for(int i=0; i<N; i++)
    {
        //只添加匹配成功的边,特征点有对应的地图点说明匹配成功
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                //mvbOutlier 的每个元素与该帧的一个特征点对应，用于标记特征点是否被视为"离群值"（outlier）
                pFrame->mvbOutlier[i] = false;

                //获得观测值
                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                //创建边
                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                //设置该边连接的顶点对象和序号
                //optimizer.vertex(0) 是用于获取优化器中的第一个顶点
                //将 optimizer.vertex(0) 返回的通用基类指针转换为g2o::OptimizableGraph::Vertex*类型的指针
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                //基于特征点的金字塔层数，构建相应信息矩阵（协方差矩阵之逆）
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                //选择较为鲁棒的Huber作为核函数
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                //给边传入内参，用来计算重投影误差和雅可比矩阵
                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                cv::Mat Xw = pMP->GetWorldPos();

                // Xw是 g2o::EdgeSE3ProjectXYZOnlyPose类型的边的成员变量，用于表示地图点的世界坐标。
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                //SET EDGE
                Eigen::Matrix<double,3,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                const float &kp_ur = pFrame->mvuRight[i];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaStereo);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                e->bf = pFrame->mbf;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
            }
        }

    }
    }

    //?多此一举,优化之前nmatches小于15，就不进行优化了
    //如果有效匹配点的数量小于3，返回0，表示无法执行姿势优化
    if(nInitialCorrespondences<3)
        return 0;

    //! step 3 开始优化，总共优化四次，每次10itr,优化后，将观测分为outlier和inlier，outlier不参与下次优化
    //* 由于每次优化后是对所有的观测进行outlier和inlier判别，因此之前被判别为outlier有可能变成inlier，反之亦然
    //* 定义的卡方阈值，卡方误差(基于重投影误差)大于阈值的被定义为外点，小于卡方误差的被定义为内点
    //* 这里和RANSAC计算F H的时候很像,只是当时区分内外点之后，还基于此给模型打分，找出最优的F,这里只是区分了内外点
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};    

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {
        //执行优化，优化10轮
        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        //设置为0，表示不限制迭代次数
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            //获取每一条边e 及 其特征点索引idx
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];
            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }
            
            //获得当前边的卡方误差,并与卡方阈值比较，区分内外点.
            //外点在之后不参与优化,但是还会比较卡房误差和阈值，满足要求还会重新添加入内点
            const float chi2 = e->chi2();
            if(chi2>chi2Mono[it])
            {                
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);  // 设置为outlier,外点在之后不参与优化
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);  // 设置为intlier
            }

            if(it==2)
            //前两次优化需要RobustKernel, 之后的不需要
                e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;     //??BUG,重新变为内点时，nBad并没有-1
            }
            else
            {                
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);
        }
        
        if(optimizer.edges().size()<10)
            break;  //break退出循环
    }    

    //! step 4 取出优化后的位姿注册到该帧中,返回去除外点的有效匹配个数
    //由于g2o的顶点estimate是李群位姿的数据格式：g2o::SE3Quat，它使用四元数来表示旋转部分，使用三维向量表示平移部分。
    //因此我们需要把他转化为OpenCV的Mat格式，并注册到pFrame对象中
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;
}

```



## （局部建图线程）局部BA优化——Optimizer::LocalBundleAdjustment

思路是通过地图点的GetObservations()方法获得所连接的关键帧
