# 推理代码调试

## 掩码预测

在**eval文件**中进行MG的掩码预测，eval文件中的形参datasets选择DAVIS，在data文件中，有关于DAVIS的配置，在这里把配置改成自己的数据。由于论文中掩码预测是绑定的RAFT光流预测算法，RAFT中还有一些参数的设置，会得到四个通道的光流文件flo和png，所以如果采用FlowFormer等别的光流预测算法，需要自己调整一下，获得文件之后需要调整到MG读取数据的流程中。

![截图 2023-06-14 22-09-30](/home/hanbing/图片/截图/截图 2023-06-14 22-09-30.png)

## 光流预测

**Predict.py中**，是使用RAFT的预训练模型。形参主要有--gap（相隔几张图像预测光流），--reverse（不清楚），--model（预训练权重），--outroot（RGB保存路径）--raw_outroot（光流保存路径）。

```python
--gap
2
--model
/home/hanbing/公共的/paper_code/motiongrouping/raft/raft-things.pth
--path
/home/hanbing/公共的/paper_code/motiongrouping/data/GT086_small45/1
--outroot
/home/hanbing/公共的/paper_code/motiongrouping/data/output/Flows_gap-2
--reverse
1
--raw_outroot
/home/hanbing/公共的/paper_code/motiongrouping/data/output/Flows_gap-2
```



# MG实验总结：

**光流预测**的不太理想，具体使用RAFT算法预测的光流，下面是RAFT中r和g参数分别取:  pip install torch==1.7.1 torchvision==0.8.2  matplotlib tensorboard scipy openc

python /home/hanbing/公共的/paper_code/The-Emergence-of-Objectness/tools/test.py  /home/hanbing/公共的/paper_code/The-Emergence-of-Objectness/configs/config_test.py  /home/hanbing/公共的/paper_code/The-Emergence-of-Objectnesth --show-dir /ho/home/hanbing/公共的/paper_code/The-Emergence-of-Objectness/output_test --work-dir /home/hanbing/公共的/paper_code/The-Emergence-of-Objectness/output_test

**r=0,g=1**

<video src="/home/hanbing/公共的/paper_code/motiongrouping/data/output/Flows_gap1/GT086_small45/vedio_1.mp4"></video>



**r=0,g=2**

<video src="/home/hanbing/公共的/paper_code/motiongrouping/data/output/Flows_gap2/GT086_small45/vedio_1.mp4"></video>

**r=1,g=1**

<video src="/home/hanbing/公共的/paper_code/motiongrouping/data/output/Flows_gap-1/GT086_small45/vedio_1.mp4"></video>



**r=1,g=2**

<video src="/home/hanbing/公共的/paper_code/motiongrouping/data/output/Flows_gap-2/GT086_small45/vedio_1.mp4"></video>

## MG图像分割

基于光流预测，通过MG的预训练模型，分割图像，作者给了四个预训练权重，其中暂时作了两个实验，davis和fbms的，其中fbms的分割结果显著好于davis，davis其中有一半的图像，将前背景搞反了。但fbms的效果也很差。

**davis效果**（mean）

<video src="/home/hanbing/公共的/paper_code/motiongrouping/results/ckpt_davis.pth/1/mean.avi"></video>

fbms效果（mean）

<video src="/home/hanbing/公共的/paper_code/motiongrouping/results/ckpt_fbms.pth/1/mean.mp4"></video>

## 实验分析

原因我认为是：1.光流估计的效果很差，这个算法是使用flow作为监督的2.在结构的设计上也没有学习任何的图像特征，仅仅提取了光流特征和光流之间的时间特征，因此该算法的效果将极大取决于光流估计的效果。