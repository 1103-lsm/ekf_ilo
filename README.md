# Aliengo 里程计：IMU 和腿部里程计

该代码库提供了 Aliengo 机器人的 IMU 和腿部里程计实现。它被设计为 ROS package，所以请确保将其放入您的 catkin 工作空间。

## 环境配置
相应的库安装脚本在 `./shell` 文件夹，分别可运行 `install.sh` 来安装库和运行 `test.sh` 来测试

## ROS Bag 数据
您可以从以下链接下载 ROS Bag 数据：
[下载 ROS Bags](https://drive.google.com/drive/folders/1x2lNNuv9gPLOuj8m5_orv0jWtfbJXJ1S?usp=sharing)

## 使用方法
首先，使用 catkin 构建 ROS package：
```bash
catkin build
```

然后，source 工作空间的 setup 文件：
```bash
source ../../devel/setup.bash
```

要运行三个 ROS bags，请使用以下指令：
```bash
roslaunch ilo run_1_bag_ilo.launch
roslaunch ilo run_2_bag_ilo.launch
roslaunch ilo run_3_bag_ilo.launch
```