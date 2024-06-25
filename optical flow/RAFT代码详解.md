# 一、demo代码

1.首先加载算法RAFT，每个步骤是什么意思还不太清楚。这里load_state_dict是加载模型的参数字典，torch.load是加载模型。![image-20230704100810320](/home/hanbing/.config/Typora/typora-user-images/image-20230704100810320.png)

2.加载路径中的所有图像，并按照两两错位排序，通过load_image转化为tensor。如果图像不能被8整除，那么利用InputPadder和pad扩充边界。最后利用model计算两个图像之间的光流，并利用viz函数可视化。

![image-20230703222203582](/home/hanbing/.config/Typora/typora-user-images/image-20230703222203582.png)

**2.1** load_image（图像路径构造为tensor），imfile是上面传入的图像路径，这里先使用PIL读取图片，之后使用array转化为numpy的narray格式，然后使用torch.from_numpy（）转化到tensor。并放到device上。

![image-20230704091459886](/home/hanbing/.config/Typora/typora-user-images/image-20230704091459886.png)

**3）**viz光流可视化操作



# 二.Model代码

**核心流程是：**对上一对图像预测光流作为本次的初始光流构造coords1，然后进入循环：通过在第二张图像的特征图中对coords1对应位置的像素相似性进行区域采样，联合上一轮的光流initflow和inp，net特征，融合后进行ConvGRU，更新coords1和隐变量net，进行可学习的上采样coords1-coords0的H/8尺度光流，得到全分辨率光流。如此迭代N次的更新，将迭代完毕最终的全分辨率光流用于测试，N次的全分辨率光流用于训练。

## step 1 特征提取

图像预处理，归一化图像和连续

![image-20230705160739146](/home/hanbing/.config/Typora/typora-user-images/image-20230705160739146.png)

前后两幅图像特征提取网络，I1和I2放在一起投入网络，权值共享。得到（2,256,H/8,H/8）维度的特征。

![image-20230705161305601](/home/hanbing/.config/Typora/typora-user-images/image-20230705161305601.png)

![image-20230705161113365](/home/hanbing/.config/Typora/typora-user-images/image-20230705161113365.png)

## step 2 初始化correlation volumes 相关性查找表

相关性查找表是相当于fmap1的每一个像素和fmap2每一个像素的点积得分（称之为相关性查找表），最终得出的L*L的大矩阵改了一下维度，然后通过平均池化下采样，把属于fmap2中的像素下采样，构成了四个尺度的相关性查找表，其中H1,W1不变，H2,W2分别是1,2,4,8倍的下采样。可以定义num_level参数从而更改采样的大小和数目。

![image-20230705161533892](/home/hanbing/.config/Typora/typora-user-images/image-20230705161533892.png)

![image-20230705162131066](/home/hanbing/.config/Typora/typora-user-images/image-20230705162131066.png)

![image-20230705162307458](/home/hanbing/.config/Typora/typora-user-images/image-20230705162307458.png)

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230705162830316.png" alt="image-20230705162830316" style="zoom: 150%;" />

## step 3 第一帧图像的特征提取网络

这里提取的特征也是（1,256,H/8,W/8）的维度。有意思的是，这里把提取出来的特征，按照channel维度切分为两个（1,128,H/8,W/8）的特征。分别定义为net和inp，并分别使用tanh和relu激活。net作为GRU隐状态而inp后面与其他特征结合作为GRU的一般输入。

![image-20230705163038500](/home/hanbing/.config/Typora/typora-user-images/image-20230705163038500.png)

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230705163720371.png" alt="image-20230705163720371" style="zoom:80%;" />

### **4. 基于上一对图像的光流结果，初始化光流坐标索引**

第一步先构建（1,2,H/8,W/8）维度的坐标网格，网格的两个维度分别代表该点的x和y坐标。初始状态在第一对图像时，coords0==coords1，在除了第一对图像之后，初始状态coords1=coords0+上一对的光流。

![image-20230706192038050](/home/hanbing/.config/Typora/typora-user-images/image-20230706192038050.png)

![image-20230705164106154](/home/hanbing/.config/Typora/typora-user-images/image-20230705164106154.png)

![image-20230705164126832](/home/hanbing/.config/Typora/typora-user-images/image-20230705164126832.png)

使用100*100的HW演示，coords0==coords1如下图所示。

<img src="/home/hanbing/.config/Typora/typora-user-images/image-20230705164358432.png" alt="image-20230705164358432" style="zoom:80%;" />

## step 5 迭代更新光流

这是比较核心的地方，内容比较多，拆分为小节。

![image-20230705164916694](/home/hanbing/.config/Typora/typora-user-images/image-20230705164916694.png)



## 进入循环

## **5.1根据coords1在相似性图表中窗口采样**

当前coords1在相似性查找表h2 w2中的坐标，对该点周围的的9*9窗口进行采样，在四个分辨率的相似性查找表中都进行采样，将采样得到的窗口赋值为corr

![image-20230705165319907](/home/hanbing/.config/Typora/typora-user-images/image-20230705165319907.png)

![image-20230705165434200](/home/hanbing/.config/Typora/typora-user-images/image-20230705165434200.png)

**核心流程是：**

1.构建对应于多尺度相似性查找表的for循环

2.获取对应尺度的相似性查找表corr

3.构建一个（9,9,2）的查找窗delta。按照自身的坐标（-4,4）排列

4.对应尺度的坐标网格和查找窗构建：

将之前构建的coords1坐标网格改为（batch h w,1,1,2）维度，这里的意思是，对于原来h1w1的每一个像素，都有一个2维度的向量表示其对应的位置，这个用来映射到查找表的h2w2中。

重要的是，需要根据当前尺度的相似性查找表除去一个变量，变量为2**i，i属于（0,3）。这里需要除去是因为当时构建coords是根据h和w的大小按照排序（0123456......）来的，也就是说，coords和正常尺度的查找表一样。当获取的是低尺度查找表的时候，需要把自身坐标值也减少到相应的大小，不然会超出索引。因此通过除去这个指数变量来与下采样对齐。从而将最后两维的位置向量与h2w2对齐。

5.将查找窗和坐标网格放在一起 coords_lvl（将原先对应的一个坐标值，变成一种9*9的查找窗）

 6.在查找表中进行采样，对于h1w1的每一个像素，根据对应位置的点采样周围9*9 81个相关性，初始化时的光流为0或上一帧的光流（当为0时，coords0==coords1,即对于h1w1每一个像素的位置在h2w2中的对应，取其周围81个像素的相关性；当光流不为0时，coords1=coords0+flow，这时对于h1w1的每一个像素，应该对应经过光流偏移之后的h2w2像素周围81个点）

![image-20230705172902365](/home/hanbing/.config/Typora/typora-user-images/image-20230705172902365.png)

把 coords_lvl分为x和y方向，然后进行归一化，将第四步尺度变化后的coords按当前尺度归为[-1,1]，之后合并。对于I1中每一个像素对应的I2所有像素来说。在网格中表示为h2,w2的左上角和右下角为（-1,-1）和（1,1）的范式。

然后进行grid_sample函数，根据搜索范围在查找表中进行采样对应特征。这个函数给定维度为(N,C,Hin,Win) 的input，维度为(N,Hout,Wout,2) 的grid，则该函数output的维度为(N,C,Hout,Wout)。对于我们的任务来说，input为（batch h1 w1, 1, h1, w1）grid为（batch h1 w1,9,9,2）由于已经进行了规范化，因此，得到对应点的（NC99）（batch h1 w1, 1, 9,9）的相关性

7.循环共得到四个分辨率的采样后的（batch h1 w1, 1, 9,9）相似性窗口，作为返回值。

### **5.2迭代求解模块**（特征融合+convgru）

将上一步帧的初始光流flow，构造好的多分辨率采样窗口corr，对第一张图像提取到的net和inp特征。共同输入进特征融合和ConvGRU模块。

![image-20230706193136370](/home/hanbing/.config/Typora/typora-user-images/image-20230706193136370.png)

**核心流程是：**

1.将corr和flow特征进行融合，作为运动特征的编码。

2.将编码的运动特征和inp特征融合，重新赋值给inp。

3.将上下文编码器提取出的net特征作为隐变量，inp作为输入，投入进ConvGRU网络中。比较特别的是，ConvGRU采用了两层结构，第一层卷积提取水平特征，第二层再提取垂直的特征。最后输出Ht作为下次迭代的隐变量net。还输出了中间更新预测的delta_flow和用于上采样的mask。

### **5.3更新 coords1 **

coords1 =  coords1+delta_flow。将coords1-coords0作为更新后的光流。

### **5.4光流上采样**

由于光流是在1/8分辨率上的特征预测的，对光流进行可学习的上采样，获得全分辨率光流flow_up，并添加进flow_predictions = []的列表，迭代轮次的参数默认为12轮,将一对图像优化得到的12个结果都构造到这个列表中，用于train，如果用于test将只使用最后一次迭代的 光流。

**核心流程是：**对上一对图像预测光流作为本次的初始光流构造coords1，然后进入循环：通过在第二张图像的特征图中对coords1对应位置的像素相似性进行区域采样，联合上一轮的光流initflow和inp，net特征，融合后进行ConvGRU，更新隐变量net，获得delta_flow和mask，进而更新coords1。使用mask进行可学习的上采样coords1-coords0的H/8尺度光流，得到全分辨率光流。如此迭代N次的更新，将迭代完毕最终的全分辨率光流用于测试，N次的全分辨率光流用于训练。

# 三.Train代码