# 代码调试

推理代码仍然没有调试好，目前的问题主要是，在data.py中的536行，x1和flow.view（-1,2）转化的大小不同无法相加。flow的维度是480，480，2，然而x1的维度是（（512*512），2），这个问题有点不太清除，可能是配置文件的原因，也有可能是我用flowformer输出的光流尺寸不对。

输入的swiss图像大小为（1024,1024,3），也就是说通过配置得到x1的HW是原图像的一半。但是经过FlowFormer计算的光流HW为（960，960，2）。这应该是FlowFormer中光流计算的问题。

将swiss的val中所有图像拿出来，并将分辨率转为3/4（768,768）进行flowformer预测，启动了flowformer的keep_size参数，得到的光流大小也为（768,768）。下一步再进行调试。

![image-20230617164243133](/home/hanbing/.config/Typora/typora-user-images/image-20230617164243133.png)