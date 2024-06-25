[TOC]



# 分别使用G-N和G2O优化位姿

位姿优化的高斯牛顿和g2o框架代码解读，并重点解读了g2o框架的内容和使用方法，代码位于SLAM14讲中的p189-195页，高博以优化一个帧的位姿[R T]为例，比较了手写高斯牛顿和使用g2o框架的区别。

## G-N和G2O的区别

G2O是通用图优化工具，只要能构建成一个图优化问题，那么就可以使用g2o框架优化，g2o框架中也封装了例如高斯牛顿和列文伯格的最小二乘优化算法。

g2o的优势在于：在G-N中，需要手写所有待优化变量的雅可比矩阵计算形式，对于需要优化大量的变量来说，需要一个个写，很麻烦。但对于g2o来说，只要它们的观测方程是同一个形式（比如SLAM的位姿优化）的话，那么就说明他们的误差类型是全部是相同的，则可以把他们统一的定义为一种类型的顶点，把这些顶点连上属于他们的那些边（误差/观测），则可以统一的计算整个雅可比矩阵。

## 一、高斯牛顿

高斯牛顿的流程非常清楚，就不多加解释了。

```c++
void bundleAdjustmentGaussNewton(
  const VecVector3d &points_3d,
  const VecVector2d &points_2d,
  const Mat &K,
  Sophus::SE3d &pose) {
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  const int iterations = 10;
  double cost = 0, lastCost = 0;
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  for (int iter = 0; iter < iterations; iter++) {
    //! H = J*J.t SIZE=6,6  
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Vector6d b = Vector6d::Zero();

    cost = 0;
    //! 每次迭代，计算i个数据点的总误差
    for (int i = 0; i < points_3d.size(); i++) {

      //! 3D-2D的投影
      Eigen::Vector3d pc = pose * points_3d[i];
      double inv_z = 1.0 / pc[2];
      double inv_z2 = inv_z * inv_z;
      Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy); //投影的u v

      //! 计算重投影误差和核函数 e是2维的
      Eigen::Vector2d e = points_2d[i] - proj;  
      cost += e.squaredNorm();

      //! 计算雅可比矩阵
      Eigen::Matrix<double, 2, 6> J;
      J << -fx * inv_z,
        0,
        fx * pc[0] * inv_z2,
        fx * pc[0] * pc[1] * inv_z2,
        -fx - fx * pc[0] * pc[0] * inv_z2,
        fx * pc[1] * inv_z,
        0,
        -fy * inv_z,
        fy * pc[1] * inv_z2,
        fy + fy * pc[1] * pc[1] * inv_z2,
        -fy * pc[0] * pc[1] * inv_z2,
        -fy * pc[0] * inv_z;

      H += J.transpose() * J;
      b += -J.transpose() * e;
    }

    Vector6d dx;
    //! 因为这里只优化了6个变量，所以可以直接线性求解，不必分解 H
    dx = H.ldlt().solve(b);

    //! 退出优化：如果H不正定导致求解dx时分母为0,不是个数 退出优化
    if (isnan(dx[0])) {
      cout << "result is nan!" << endl;
      break;
    }

    //! 退出优化：如果这一轮的cost>=上一轮的说明优化到头了已经到最低点了
    if (iter > 0 && cost >= lastCost) {
      // cost increase, update is not good
      cout << "cost: " << cost << ", last cost: " << lastCost << endl;
      break;
    }

    //! 更新pose
    pose = Sophus::SE3d::exp(dx) * pose;
    lastCost = cost;

    cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << endl;

    //! 当dx足够小，认为迭代到最低点了
    if (dx.norm() < 1e-6) {
      // converge
      break;
    }
  }

  cout << "pose by g-n: \n" << pose.matrix() << endl;
}
```



## 二、g2o框架

![img](https://pic3.zhimg.com/v2-fb35b76967fb6a81bdb35b17aa334d26_r.jpg)

### step 1 重写顶点类

重写顶点最重要的是**继承的顶点类型**、**优化变量的初始值设置**和**优化变量的更新计算**；

**其中对于顶点类型**，g2o给了一些预定义类型，包括类型其中的函数，也是g2o针对slam问题定义好的，在我们SLAM中做BA优化的时候，可以直接调用。而如果是g2o里面没有预定以好的，我们就要继承g2o::BaseVertex并重写BaseVertex<D ，T>作为新的顶点类。其中D代表待优化变量的维度，T代表待优化变量的类型。 

g2o中已经定义的一些顶点类如下：**一般来说SLAM任务中的顶点和边类型，g2o都定义好了**，但也可以自己重写，就像下面高博教材里面的一样。



```cpp
                                    G2O中已经定义的顶点
//括号内的表示他们在g2o中已经继承和重写好的BaseVertex<D,T>

g2o::ertexSE2  (class ertexSE2 : public BaseVertex<3, SE2>)
//2D pose Vertex, (x,y,theta)

g2o::VertexSE3 (class VertexSE3: public BaseVertex<6, Isometry3>) 
//Isometry3使欧式变换矩阵T，实质是4*4矩阵
//6d vector (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion)

g2o::VertexPointXY (class VertexPointXY: public BaseVertex<2, Vector2>)
g2o::VertexPointXYZ (class VertexPointXYZ : public BaseVertex<3, Vector3>)
g2o::VertexSBAPointXYZ (class VertexSBAPointXYZ: public BaseVertex<3, Vector3>)
//g2o::VertexSBAPointXYZ  SLAM中优化的地图点就是用的这个顶点
    
    
/* 使用SE3李群位姿，他包括了平移和旋转变换，SLAM中优化的位姿就是用的这个顶点
 SE3 Vertex 在内部使用变换矩阵进行参数化，在外部使用其指数图进行参数化*/
g2o::VertexSE3Expmap (class: public BaseVertex<6, SE3Quat>)

    
/*表示相机参数的顶点，优化变量的维度是6，这包括了相机内外参数的各个自由度。
这个顶点主要用于相机的参数优化，例如相机的焦距、畸变系数等。*/
g2o::VertexCam (class: public BaseVertex<6, SBACam>)

    
/*
使用的位姿表示方式是Sim3，它包括了平移、旋转和尺度变换。
优化变量的维度是7，这包括了刚体变换（SE3）的6个自由度和一个尺度因子。*/
g2o::VertexSim3Expmap  (class : public BaseVertex<7, Sim3>)
//Sim3 Vertex, (x,y,z,qw,qx,qy,qz),7d vector,(x,y,z,qx,qy,qz) (注意省略了四元数的 w 部分）
```



除了顶点类型之外，比较重要的是顶点的两个函数复写，一个是赋予待优化变量初始值的setToOriginImpl() 和优化变量的更新计算的oplusImpl(update)。

**重写顶点代码**

```c++
//! step 1 重写顶点类

class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> { //继承并复写函数g2o::BaseVertex<D， T > 前面代表优化变量的维度，后面代表优化变量的类型

public:
  //解决Eigen的内存分配问题，反正加上就行
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  //! 1.1复写函数：分配待优化变量的初始值
  virtual void setToOriginImpl() override {
    //设置为 Sophus::SE3d()，表示初始值为一个单位刚体变换
    _estimate = Sophus::SE3d();
  }

  //! 1.2复写函数：优化变量的更新计算
  // Sophus::SE3d表示李群位姿，其增量扰动模型的更新的公式为：exp(update_eigen) * _estimate;
  virtual void oplusImpl(const double *update) override {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }
  //! 1.3复写函数：读写。
  //如果没有读写需求可以像这样空着或者直接不写
  virtual bool read(istream &in) override {}
  virtual bool write(ostream &out) const override {}
};
```



### step 2 重写边类

**对于边的类型**，`BaseUnaryEdge`，`BaseBinaryEdge`，`BaseMultiEdge` 分别表示一元边，两元边，多元边。

顾名思义，一元边可以理解为一条边只连接一个顶点，两元边理解为一条边连接两个顶点（常见），多元边理解为一条边可以连接多个（3个以上）顶点。在SLAM中，同时优化地图点和位姿用的是二元边，只优化位姿用的是一元边。拿一元边来距离，g2o::BaseUnaryEdge<D,E,Vertextype> D,E,Vertextype分别表示其误差维度，误差类型，和连接的顶点类型。

除此之外，**g2o中也针对slam任务定义了一些边类**，比如`g2o::EdgeSE3ProjectXYZ` 、`g2o::EdgeSE3ProjectXYZOnlyPose`和`g2o::EdgeStereoSE3ProjectXYZOnlyPose`

1. `g2o::EdgeSE3ProjectXYZ`：（同时优化位姿和地图点）
   - 用于表示三维地图点在三维世界坐标系中的位置与其在二维图像平面上的投影之间的约束。
   - 通常与相机位姿、地图点的位置、图像特征点的像素坐标等相关联。
   - 可用于同时优化相机位姿和地图点的位置。
2. `g2o::EdgeSE3ProjectXYZOnlyPose`：(仅优化位姿)
   - 用于表示三维地图点在三维世界坐标系中的位置与其在二维图像平面上的投影之间的约束，但只考虑相机位姿的优化，不考虑地图点的优化。
   - 通常在一些场景中，地图点的位置已知或不需要优化，只需要优化相机位姿时使用。
   - 适用于在视觉SLAM中进行位姿估计或位姿优化。
3. `g2o::EdgeStereoSE3ProjectXYZOnlyPose` (立体视觉下仅优化位姿)
   - 用于表示在立体视觉SLAM中，三维地图点在三维世界坐标系中的位置与其在立体相机图像平面上的投影之间的约束关系
   - 通常在一些场景中，地图点的位置已知或不需要优化，只需要优化相机位姿时使用。
   - 适用于在视觉SLAM中进行位姿估计或位姿优化。



当然也完全可以继承一元二元等边自己重写，重写边比较重要的是**边的类型**、重写构造函数，计算Error函数和计算雅可比矩阵的函数。

对于边来说，一般如果是SLAM只优化位姿T，而不优化地图点，是需要重新写构造函数的，因为在重写计算Error函数时，需要读取K和pos3d的。即便我们可以把K写在类中，但由于pos3d在之后每一次添加边的时候都会发生变化，所以只能在每次添加边的时候传入对象中，所以就需要写个构造函数去接收。并且如果写了构造函数还需要再写一个private 把pos3d放进去。

```c++
EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}

······

private:
  Eigen::Vector3d _pos3d;
  Eigen::Matrix3d _K;
```

**重写边代码**

```c++

//! step 2 重写边类
//继承并复写一元边g2o::BaseUnaryEdge<D,E,VertexPose> D,E,VertexPose分别表示其误差维度，误差类型，和连接的顶点类型（注意如果重写了，是重写之后自己定义的类）
class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  //重写一下构造函数用来初始化，这里带入了两个参数pos K 放入了新定义的成员变量_pos3d和_K中，在后面也要在private中定义
  EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}
  
  //! 2.1复写函数：计算误差
  virtual void computeError() override {
    //每次计算误差的时候，把该边的第一个顶点掏出来，用这个顶点指针访问当前的变量值进行计算
    //_vertices是BaseUnaryEdge类储存顶点信息的成员变量，_vertices的大小为该边所连接的顶点数，比如二元边该成员变量size为2
    const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
    //获得当前的优化变量 T 
    Sophus::SE3d T = v->estimate();
    //获得当前重投影[u v 1]
    Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
    pos_pixel /= pos_pixel[2];
    //取重投影的前两位，也就是uv计算重投影误差
    _error = _measurement - pos_pixel.head<2>();
  }
  //! 2.2写函数：计算雅可比矩阵
  //单独pose的雅可比矩阵计算在14讲的186页，这就是把公式的结果用代码写了一遍
  virtual void linearizeOplus() override {
    const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_cam = T * _pos3d;
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double cx = _K(0, 2);
    double cy = _K(1, 2);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Z2 = Z * Z;
    _jacobianOplusXi
      << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
      0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
  }

  //! 2.3复写函数：读写。
  //如果没有读写需求可以像这样空着
  virtual bool read(istream &in) override {}
  virtual bool write(ostream &out) const override {}

//! 2.4定义构造函数中新增的成员变量
private:
  Eigen::Vector3d _pos3d;
  Eigen::Matrix3d _K;
};
```



### step 3 构建图模型，开始优化

#### step 3.1 构建Blocksolver、Linearsolver

**Block：**g2o预定义了以下几种常用类型，使用这些类型相当于g2o::BlockSolver < g2o::BlockSolverTraits<6,3> > 或g2o::BlockSolver < g2o::BlockSolverTraits<7,3> > 和g2o::BlockSolver < g2o::BlockSolverTraits<3,2> >。

- **g2o::BlockSolver_6_3 ：表示pose 是6维，观测点是3维，用于3D SLAM中的BA。**(SLAM中最常用的是这个)
- g2o::BlockSolver_7_3：在BlockSolver_6_3 的基础上多了一个scale。
- g2o::BlockSolver_3_2：表示pose 是3维，观测点是2维。



**Linear：**这一步中我们可以选择不同的求解方式来求解线性方程  ，g2o中提供的求解方式主要有：

- LinearSolverCholmod ：使用sparse cholesky分解法，继承自LinearSolverCCS。
- LinearSolverCSparse：使用CSparse法，继承自LinearSolverCCS。
- LinearSolverPCG ：使用preconditioned conjugate gradient 法，继承自LinearSolver。
- LinearSolverDense ：使用dense cholesky分解法，继承自LinearSolver。
- LinearSolverEigen： 依赖项只有eigen，使用eigen中sparse Cholesky 求解，因此编译好后可以方便的在其他地方使用，性能和CSparse差不多，继承自LinearSolver。
- 

#### step 3.2 创建优化器对象

在这一步，我们可以选择牛顿高斯，列文伯格和doglog的方法，创建优化器需要指定Block对象，也就是说一般来说是先创建Block对象，然后再用Block初始化优化器。也可以先设定Block和Linear的类而不创建，随后在创建优化器的时候同时把Block在括号里直接创建了。如下面高博的代码。

- g2o::OptimizationAlgorithmGaussNewton
- g2o::OptimizationAlgorithmLevenberg 
- g2o::OptimizationAlgorithmDogleg 

#### step 3.3 之后的直接看下面的代码

创建顶点和边需要设置ID，顶点初始值，边的观测值等等，可以使用循环的方式赋值。

```c++
//! step 3 构建图优化函数
void bundleAdjustmentG2O(
  const VecVector3d &points_3d,
  const VecVector2d &points_2d,
  const Mat &K,
  Sophus::SE3d &pose) {

  //! 3.1 Blocksolver、Linearsolver
  //问题？？？BlockSolverTraits<M,N>的参数为6和3,查了国外的文档，M，N分别为两个顶点块的维度，但是这里只有一个顶点呀，哪里有P作为顶点？？
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;  // pose is 6, landmark is 3
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
  //! 3.2 创建优化器对象，选择梯度下降方法，可以从GN, LM, DogLeg 中选
  //make_unique 用于创建动态分配的 std::unique_ptr 对象，并返回一个指向这个对象的智能指针，对象的类型是T,make_unique<T>
  //创建优化器实例的时候需要在括号内指定Block初始化，而Block在创建实例的时候由于需要LinearSolver来初始化。
  //所以这里高博直接在创建优化器的时候，在括号内全部创建实例，但我不是很喜欢这样写，但作为一种框架，可以直接拿来用。
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

  //! 3.3 创建图模型，并把优化器放进去
  g2o::SparseOptimizer optimizer;     // 图模型
  optimizer.setAlgorithm(solver);   // 设置求解器
  optimizer.setVerbose(true);       // 打开调试输出

  //! 3.4 创建顶点对象并添加到图模型中
  VertexPose *vertex_pose = new VertexPose(); // 实例化顶点对象  
  vertex_pose->setId(0);                      // 设置每个顶点序号
  vertex_pose->setEstimate(Sophus::SE3d());   // 设置初始估计值
  optimizer.addVertex(vertex_pose);           // 添加到图模型中

  // K 传入内参，边类中计算误差需要
  Eigen::Matrix3d K_eigen;
  K_eigen <<
          K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
    K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
    K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

  //! 3.5 创建边对象并添加到图模型中
  // 边就相当于误差，这里要把所有的数据都传进去计算总error，算作一次epoch
  int index = 1;
  for (size_t i = 0; i < points_2d.size(); ++i) {
    auto p2d = points_2d[i];
    auto p3d = points_3d[i];
    EdgeProjection *edge = new EdgeProjection(p3d, K_eigen); //创建n个边
    edge->setId(index);                       //给每一条边设置序号（从1开始，所以用index）
    edge->setVertex(0, vertex_pose);          //设置每条边的第一个顶点为vertex_pose（因为这个例子中只对一个帧的位姿优化了，所以只有一个顶点）
    edge->setMeasurement(p2d);                //设置每一条边的观测值
    edge->setInformation(Eigen::Matrix2d::Identity()); //设置信息矩阵（协方差矩阵之逆）计算增量的时候使用
    optimizer.addEdge(edge);                            //将边添加到图模型中
    index++;
  }
//! 3.6 初始化优化器并指定优化次数
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();//记录时间
  optimizer.setVerbose(true);         //打开调试输出
  optimizer.initializeOptimization(); //初始化优化器
  optimizer.optimize(10);             //优化10次
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
  cout << "pose estimated by g2o =\n" << vertex_pose->estimate().matrix() << endl;
  pose = vertex_pose->estimate();      //获取顶点的当前估计值
}
```

