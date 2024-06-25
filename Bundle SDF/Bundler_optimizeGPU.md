# 2.14第四步：在GPU上局部BA优化——Bundler::optimizeGPU

​		**这是2.14的第四步局部优化部分**，传入参数是已经找好和两两匹配好的局部帧。

![image-20231031163802779](/home/hanbing/公共的/typora实验记录/Bundle SDF/assets/image-20231031163802779.png)

​		到了这一步，我们已经得到待优化的局部帧位姿和每一帧的匹配关系，理论上我们只需要基于匹配关系找到每一个地图点的共视关系，就可以通过g2o框架来优化了。但是作者这里写的十分复杂，函数的嵌套很多，我们先按照作者的思路来梳理优化过程。

## step 1 从配置中获取优化参数

​		获取外部迭代次数 7、内部迭代次数 5、newframe最小匹配特征数 15 等。

## step 2  把每个帧对匹配信息存入变量中

​		`global_corres`用来保存每对帧之间的匹配信息构造的对象corres，corres的成员变量就是匹配信息，包括帧的索引、匹配点的法线向量、3D位置信息等。`n_match_per_pair`存储每对帧之间的特征匹配数量。在这其中，如果如果检查等级>=3，还会可视化每两帧之间的特征匹配。

<img src="/home/hanbing/公共的/typora实验记录/Bundle SDF/assets/image-20231101112428178.png" alt="image-20231101112428178" style="zoom:80%;" />

## step 3  使用每个帧gpu内存上的图像数据构造变量

​		依次把每一帧的彩色图 深度图 、法线向量和位姿pose 插入`depths_gpu` , `colors_gpu` ,`normals_gpu`，`poses`变量，这些变量后续会通过下采样缓存到CUDACache类中，用于之后使用CUDA加速局部BA。同时更新update_pose_flags标识，初始帧和已经过nerf优化后的帧不优化位姿。

## step 4 如果检查等级>=4 ,保存优化之前的局部帧点云

![image-20231101164613383](/home/hanbing/公共的/typora实验记录/Bundle SDF/assets/image-20231101164613383.png)

## step 5 CUDA加速优化，cuda_cache下采样后SBA和CUDASolverBundling优化位姿

**第一步：**保存优化前的相机位姿，同时把每一帧的位姿poses放在gpu内存中，用d_transforms表示。

<img src="/home/hanbing/公共的/typora实验记录/Bundle SDF/assets/image-20231101165950853.png" alt="image-20231101165950853" style="zoom:67%;" />

**第二步：**统计每一帧的匹配数量放入n_corres_per_frame，并找出局部帧中匹配点最多的数目放入max_corr_per_image

**第三步：**初始化CUDACache类，并使用CUDA核函数将下采样的数据缓存到cuda_cache对象

<img src="/home/hanbing/公共的/typora实验记录/Bundle SDF/assets/image-20231101100204793.png" alt="image-20231101100204793" style="zoom:85%;" />

​		**3.1：初始化CUDACache类的对象cuda_cache**。 根据配置的下采样倍数，使用下采样之后的宽高WH和局部帧数量初始化CUDACache类的对象cuda_cache，构造函数中初始化了CUDACache类的成员变量，包括原始和下采样后的图像宽高，内参矩阵K以及滤波的参数等，==详见下面的参数列表和（CUDACache::cuda_cache）== 。

​		**3.2：缓存下采样的帧数据到cuda_cache对象。 **循环使用storeFrame函数缓存每一个局部帧的数据，在该函数中，通过不同的核函数将帧的数据（4D坐标向量、深度图、彩色图和法线向量）下采样并储存到CUDACache类的m_cache向量中，m_cache向量的索引就对应着局部帧的索引，索引的元素表示一种特殊的帧类型，命名为CUDACachedFrame 。也就是说，这个帧类型的每一个对象的成员变量即对应着每一帧下采样之后的（4D坐标向量、深度图、彩色图和法线向量）数据。比如，m_cache.[0].d_cameraposDownsampled就可以获取第0个局部帧的下采样之后的4D齐次坐标向量。这样一来，我们就通过CUDACache类获得了每一帧在GPU上下采样后的数据。==详见（CUDACache::storeFrame）==。

​		**为什么要下采样？**我的直觉和GPT的回答是用于减小图像或点云数据的大小，降低计算和内存需求，同时保留足够的信息以支持后续处理任务。但是在BA优化环节，只需要知道3D点的共视关系和每一帧的位姿就行了，为什么需要深度图，彩色图等等这些，为了稠密优化？

​		**为什么要使用CUDA核函数下采样？**下采样本本身是一种对图像等3维数据的处理，使用CUDA核函数能够借助并行线程加速。

**第四步：初始化SBA和CUDASolverBundling类** —— SBA::SBA ， CUDASolverBundling::CUDASolverBundling

<img src="/home/hanbing/公共的/typora实验记录/Bundle SDF/assets/image-20231101100326257.png" alt="image-20231101100326257" style="zoom:70%;" />

​		初始化SBA类的对象sba，根据配置文件和之前获得的变量初始化成员变量，这些变量用于传入CUDASolverBundling类。在sba初始化的内容中又初始化了CUDASolverBundling类的对象m_solver 用于后续的优化。

**第五步： 执行稀疏BA优化**——SBA::align， CUDASolverBundling::solve

​		首先进入SBA::align函数，在该函数中根据初始化的sba成员变量再构造一些变量，比如把每一帧的位姿解耦为R 和 t 储存在d_xRot, d_xTrans变量中等。然后调用CUDASolverBundling.m_solver求解器优化位姿。 CUDASolverBundling.m_solver-> solve的优化内容代码看起来有点费劲，这里需要知道是使用CUDA的核函数加速稀疏BA优化过程。

==其中CUDASolverBundling.m_solver -> solve（）这一部分我真的没看懂，到底是使用什么框架在优化，CUDA加速编程，等把代码的总体框架看完之后，就回头来看这个CUDA加速的部分，我觉得真有用，暂时先看到SBA这一层就行，下面的CUDA先不看==

**第六步：保存优化之后的位姿**

<img src="/home/hanbing/公共的/typora实验记录/Bundle SDF/assets/image-20231101165908347.png" alt="image-20231101165908347" style="zoom:67%;" />

## step 6 检查位姿变化是否过大

​		如果当前帧和其参考帧是相邻帧，那么需要检查位姿变化是否过大。具体来说，检查位姿变化：如果平移变化或旋转变化超过了预定义的阈值 max_trans 或 max_rot，则认为 _newframe 的位姿变化异常，将 _newframe 的状态标记为 FAIL。

## step 7  更新优化后的位姿 , 保存优化之后的局部帧点云

​		把优化后的位姿从poses更新到Frame._pose_in_model中。（由于poses变量是表示这一序列的局部帧的位姿列表，在更新完之后需要把位姿更新到每一个Frame对象的属性中）把优化之后的帧的点云保存为PLY文件。



## 参数列表

| Bundler OptimizerGpu                 | 类型                            | 意义                                                         |
| ------------------------------------ | ------------------------------- | ------------------------------------------------------------ |
| depths_gpu                           | std::vector<float*>             | 储存所有局部帧深度图的向量                                   |
| colors_gpu                           | std::vector<uchar4*>            | 储存所有局部帧彩色图的向量                                   |
| normals_gpu                          | std::vector<float4*>            | 储存所有局部帧点云法线向量图的向量                           |
| poses                                | std::vector< Eigen::Matrix4f>   | 储存所有局部帧poses的向量                                    |
| n_match_per_pair                     | std::vector< int>               | 存储每对帧之间的特征匹配数量的向量                           |
| global_corres                        | std::vector< EntryJ>            | 保存每对帧之间的每一对匹配点的对象corrrs (对象的成员变量包括两个帧的索引、匹配点的法线向量、3D位置信息等匹配信息)，向量的长度表示所有的匹配点对数目<br />例如： global_corres[i].imgIdx_i |
| update_pose_flags                    | std::vector< int>               | 控制该帧位姿是否优化的标志，初始帧和已经过nerf优化后的帧该标识为0 |
| d_transforms                         | float4x4*                       | GPU内存中的每一帧的pose                                      |
| n_corres_per_frame                   | std::map<int,int>               | 储存每一帧的匹配数量的字典                                   |
| max_corr_per_image                   | int                             | 匹配点最多的数目                                             |
| max_n_residuals                      | int                             | 最大残差个数？？                                             |
|                                      |                                 |                                                              |
| **CUDACache类**                      | **类型**                        | **意义**                                                     |
| m_width                              |                                 | 下采样之后的图像宽                                           |
| m_height                             |                                 | 下采样之后的图像高                                           |
| m_maxNumImages                       |                                 | 局部帧的数目                                                 |
| m_intrinsics                         |                                 | 下采样调整后的内参矩阵K                                      |
| m_intrinsicsInv                      |                                 | 下采样调整后的内参矩阵K之逆                                  |
| m_inputDepthWidth                    |                                 | 原始的图像宽                                                 |
| m_inputDepthHeight                   |                                 | 原始的图像高                                                 |
| m_inputIntrinsics                    |                                 | 原始的内参矩阵K                                              |
| m_inputIntrinsicsInv                 |                                 | 原始的内参矩阵K之逆                                          |
| m_filterIntensitySigma               |                                 | 滤波的参数，用于之后的深度图或点云的处理                     |
| m_filterDepthSigmaD                  |                                 | 滤波的参数，用于之后的深度图或点云的处理                     |
| m_filterDepthSigmaR                  |                                 | 滤波的参数，用于之后的深度图或点云的处理                     |
| m_currentFrame                       |                                 | 初始化为 0，在CUDACache::storeFrame时作为储存帧的数据到m_cache中的索引。 |
| 👆👆👆👆👆👆                               | 👆👆👆👆👆👆                          | ==上面这些变量是CUDACache构造函数时初始化的==                |
| 👇👇👇👇👇👇                               | 👇👇👇👇👇👇                          | ==下面这些变量是CUDACache::storeFrame之后初始化的==          |
| m_cache                              | std::vector < CUDACachedFrame > | 储存每一个局部帧的数据，向量中的每一个元素，按顺序对应着每一个局部帧中的数据 |
|                                      |                                 | 可以通过m_cache.[int].d_cameraposDownsampled的方式索引到某个局部帧对应的数据。下面这些变量都是 CUDACachedFrame类中的成员变量，因此CUDACachedFrame可以看作是某种意义上的帧类。 |
| d_helperCamPos                       | float4*                         | 原始尺度的深度图depths_gpu转化的4D齐次坐标向量               |
| d_cameraposDownsampled               | float4*                         | d_helperCamPos下采样之后的4D齐次坐标向量                     |
| d_normalsDownsampled                 | float4*                         | 下采样之后的法线向量normals_gpu                              |
| d_depthDownsampled                   | float*                          | 下采样之后的深度图depth_gpu                                  |
| d_num_valid_points                   | int*                            | 深度图中>0.1m的有效个数                                      |
|                                      |                                 |                                                              |
| **SBA类**                            | **类型**                        | **意义**                                                     |
| maxNumIts                            | int                             | 稀疏BA的最大迭代次数                                         |
| m_localWeightsSparse                 | std::vector< float>             | 局部权重稀疏？？                                             |
| m_localWeightsDenseDepth             | std::vector< float>             | 局部权重稠密深度图？？                                       |
| m_localWeightsDenseColor             | std::vector< float>             | 局部权重稠密彩色图？？                                       |
| m_maxResidual                        | float                           | 用于存储最大残差的变量，初始化为-1.0f                        |
| m_bUseGlobalDenseOpt                 | bool                            | 是否在 Bundle Adjustment 中使用全局的稠密优化                |
| m_bUseLocalDense                     | bool                            | 是否在 Bundle Adjustment 中使用局部的稠密优化。              |
| m_bUseComprehensiveFrameInvalidation | bool                            | 不知道干啥的？？，初始化false                                |
| d_xRot                               | float3*                         | 放在GPU上的用于储存每一帧的旋转矩阵R                         |
| d_xTrans                             | float3*                         | 放在GPU上的用于储存每一帧的位移t                             |
| maxNumImages                         | int                             | 局部帧的个数                                                 |
| m_bVerify                            | bool                            | 不知道干啥的？？，初始为false                                |
| 👆👆👆👆👆👆                               | 👆👆👆👆👆👆                          | ==上面这些变量是SBA构造函数中初始化的==                      |
| 👇👇👇👇👇👇                               | 👇👇👇👇👇👇                          | ==上面这些变量是SBA::align中初始化的==                       |
| usePairwise                          | bool                            | 初始化为true，稠密可选参数？？                               |
| numImages                            | int                             | 表示局部帧的数目                                             |
| d_correspondences                    | std::vector< EntryJ>            | 在GPU内存上缓存的global_corres，表示所有局部帧的匹配信息，向量长度为所有匹配的数目 |
| m_numCorrespondences                 | int                             | 局部帧所有匹配的数目                                         |
| d_validImages                        | std::vector< int>               | 在GPU内存上的向量，包含了每个局部帧的有效性标志，向量长度为局部帧的个数 |
| cache                                | CUDACache*                      | 指向第三步初始化好的&cuda_cache对象                          |
| weightsSparse                        | std::vector< float>             | = m_localWeightsSparse （不太明白为什么要重新赋值，都在一个类中，继续用这个成员变量不行吗） |
| weightsDenseDepth                    | std::vector< float>             | = m_localWeightsDenseDepth                                   |
| weightsDenseColor                    | std::vector< float>             | = m_localWeightsDenseColor                                   |
| revalidateIdx                        | int                             | 传入align为-1，不知道是啥意思？？？                          |
|                                      |                                 |                                                              |
| **CUDASolverBundling类**             | **类型**                        | **意义**                                                     |
|                                      |                                 |                                                              |





