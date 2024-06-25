# 0.下载

在官方网站安装Docker 引擎 https://docs.docker.com/engine/install/ubuntu/#set-up-the-repository

## 0.1设置存储库

更新`apt`包索引并安装包以允许`apt`通过 HTTPS 使用存储库：

```bash
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
```

添加Docker官方GPG密钥：

```bash
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
```

使用以下命令设置存储库：

```bash
echo "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu" $(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

## 0.2安装Docker引擎

更新`apt`包索引：

```bash
sudo apt-get update
```

安装 Docker 引擎、containerd 和 Docker Compose

```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

通过运行镜像来验证Docker Engine安装是否成功 `hello-world`

```bash
sudo docker run hello-world
```

## 0.3安装Docker desktop

下载最新的[DEB包](https://desktop.docker.com/linux/main/amd64/docker-desktop-4.21.1-amd64.deb?utm_source=docker&utm_medium=webreferral&utm_campaign=docs-driven-download-linux-amd64&_gl=1*16hs50o*_ga*MTc1NDU0MTUwMC4xNjg5NTYyMTg1*_ga_XJWPQMJYHQ*MTY4OTU2MjE4NC4xLjEuMTY4OTU2MjMyNy4zOS4wLjA.)

使用 apt 安装软件包，如下所示：

```bash
sudo apt-get update
chmod 777 ./docker-desktop-<version>-<arch>.deb
sudo apt-get install ./docker-desktop-<version>-<arch>.deb
```
