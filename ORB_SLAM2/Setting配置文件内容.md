# 配置文件.YAML

以RGBD tum配置文件为例

```yaml

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------

# 相机标定和畸变参数（OpenCV）
Camera.fx: 517.306408
Camera.fy: 516.469215
Camera.cx: 318.643040
Camera.cy: 255.313989

#k1 k2 k3 p1 p2分别为相机的径向、切向畸变的校正系数
Camera.k1: 0.262383
Camera.k2: -0.953104
Camera.p1: -0.005358
Camera.p2: 0.002628
Camera.k3: 1.163314

# 图像分辨率
Camera.width: 640
Camera.height: 480

# 相机每秒帧数
Camera.fps: 30.0

# baseline * fx
Camera.bf: 40.0

# 图像的颜色顺序（0：BGR，1：RGB。如果图像是灰度则忽略）
Camera.RGB: 1

# Close/Far threshold. Baseline times.
ThDepth: 40.0

# Deptmap values factor 
DepthMapFactor: 5000.0

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# 特征点个数
ORBextractor.nFeatures: 1000 

# 特征金字塔的尺度因子
# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2
# 特征金字塔的层数
# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# 图像被划分为网格。在每个单元中提取 FAST，施加最小响应。
# 首先我们施加iniThFAST。如果没有检测到角点，我们会施加较低的值 minThFAST
# You can lower these values if your images have low contrast			
ORBextractor.iniThFAST: 20  #初始 FAST 特征提取阈值
ORBextractor.minThFAST: 7   #最小 FAST 特征提取阈值

```

