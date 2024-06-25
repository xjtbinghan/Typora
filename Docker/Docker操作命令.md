残差全连接

## 从docker Hub上拉取镜像后，在该镜像的基础上新安装库

docker run -it (镜像名)  在容器中运行镜像

sudo apt-get update

sudo apt-get install () 或者

sudo pip install () 

保存到新的镜像（如果不保存该镜像的操作会全部退回到没有打开之前）：

docker commit  (+当前运行的容器名字) （保存后的新镜像名，首字母不可以大写）

## 

**宿主机和容器内文件互相拷贝的方法**

https://blog.csdn.net/dongdong9223/article/details/71425077

docker build  **建立镜像**

dockedocker images **查看镜像**

docker pull **从dockerhub上拉取镜像**

docker push **把镜像push到dockerhub上**

docker ps -a **查看所有容器**

docker ps  **查看运行的容器**

docker rm e1d0d9e2a2d7 或 docker rm -f e1d0d9e2a2d7 **删除容器**

docker run -it 镜像名 /bin/bash  **运行容器**

docker restart 容器ID     **重启容器**

docker stop containerId   **停止运行容器**  containerId 是容器的ID

docker stop $(docker ps -a -q)    **停止所有容器** 

docker rm $(docker ps -a -q)       **删除所有容器**

![image-20230717145605785](/home/hanbing/.config/Typora/typora-user-images/image-20230717145605785.png)