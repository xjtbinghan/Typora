# 关于环境的问题

## CUDA版本问题

配置环境出问题，但从问题中发现了，本机的cuda版本太高了，ubuntu的版本也高。去NVIDIA官网上下载了CUDA，22.04能下的也就是cuda11.7往上，安装了一个cuda11.8，之后用环境变量指向了11.8。然后配套下载了cudnn。cudnn下载完之后是按照官网的教程步骤操作的，没有采用网上的博主下载压缩包解压后把几个文件放进cuda里。但好像demo环境能跑了。cuda版本和cudnn版本对于tensorflow的安装很重要。