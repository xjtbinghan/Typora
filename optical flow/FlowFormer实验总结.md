# 推理代码调试

## flow可视化（flow-RGB）

在visualize_flow文件中设置seq_dir设置自定义路径，图片的格式必须是000001.png序列 如果是jpg要把上面的generate_pairs中改称jpg，形参要把 --eval_type  sqe加上，这样才是预测自己数据集的光流。模型的配置在build model函数里面，对应着config文件中的submission.py文件，可以在CN.model中指定选择的预训练权重。目前是sintel最好。

![截图 2023-06-14 21-59-24](/home/hanbing/图片/截图/截图 2023-06-14 21-59-24.png)

## flo文件生成

在visualize_flow文件中，最重要的一个函数为visualize_flow,该函数包括构造两两的图像读取顺序和最终的flo、png路径，计算光流，光流可视化，图像写入。发现在compute_flow函数计算完光流后得到的[H,W,2]的光流张量并未进行保存，于是新增了一个writeFlowFile的函数（从运动分组的Pridict.py中抄过来的），用于将[H,W,2]的光流张量保存。同时在原有的prepare_image函数中定义了保存光流的路径，和可视化之后的图像一样，将png改为flo。



# 推理结果：

FlowFormer的预训练模型推理结果:可以发现不同训练集下的结果差异非常之大，这就说明目前的Flow预测算法大多都还没有提取到通用的特征，**下面有两个思路：找一个无监督的通用大模型在我的数据集上微调，或是用无监督模型自己训练，目的是提取空间的环境特征。**

**Kitti预训练权重**

<video src="/home/hanbing/公共的/paper_code/FlowFormer-Official/demo_data/GT086_small45_kitti/kitti_pre.mp4"></video>

**Sintel预训练权重**

<video src="/home/hanbing/公共的/paper_code/FlowFormer-Official/demo_data/GT086_small45_sintel/sintel_pre.mp4"></video>

**chair预训练权重**

<video src="/home/hanbing/公共的/paper_code/FlowFormer-Official/demo_data/GT086_small45_chair/chair_pre.mp4"></video>

**things预训练权重**

<video src="/home/hanbing/公共的/paper_code/FlowFormer-Official/demo_data/GT086_small45_things/things_pre.mp4"></video>



**在SwissCube数据集上**测试的效果，意想不到的好：

![image-20230617143715419](/home/hanbing/.config/Typora/typora-user-images/image-20230617143715419.png)

但是，Swiss的数据是1024*1024的，在visualize_flow上会报错，于是把分别率缩小到了3/4（768,768），并把swiss的val数据集中的图像全部整个到了一起。放进flow_former上面预测一下光流。--keep_size打开了，光流也是768x768的

![000210](/home/hanbing/公共的/datasets/2021 Swiss Cube satellite dataset/SwissCube_1.0/flowpredict_Flow_Former_34/SwissCube_val_res34/000210.png)



**Stream2的所有val数据**进行了一次推理，Sintel预训练权重，结果如下：

<video src="/home/hanbing/公共的/paper_code/FlowFormer-Official/demo_data/hebing_Stream_val_image/Stream_val.mp4"></video>

<video src="/home/hanbing/公共的/paper_code/FlowFormer-Official/demo_data/hebing_Stream_val/Sintel_Stream_val.mp4"></video>