# 第五节 编译ROS Package

这一节来学习编译（build）ROS的package。官方文档请点[这里](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)。

### 1. 编译packages

如果没有把```source /opt/ros/melodic/setup.bash```添加到 ~/.bashrc文件的话，先在打开的终端运行

```bash
source /opt/ros/melodic/setup.bash
```

如果你跟着之前的教程一步一步来的话，打开我们的工作目录 ~/catkin_ws/，catkin_make 命令编译。命令为：

```bash
cd ~/catkin_ws
catkin_make
```

依次运行以上命令，就完成了ROS包的编译。这是回到 ~/catkin_ws/ 目录，就会看到多了一个 build/ 文件夹，所有编译的结果都在 build/ 文件夹里，其他文件不受影响。这时会编译该工作空间（workspace）下的所有packages。

```
cd .. #之前在 ~/catkin_ws/src
ls
```

结果类似如下：

![](../images/ros_build.png)

我们后面要的一些库（libraries）和可执行文件都在 build/ 里面。

### 2. 总结

catkin_make 在支持标准的 cmake，并在上面略微增加了一些ROS需要的内容。使用起来也很简单，就一句命令。

下面开始介绍ROS节点。