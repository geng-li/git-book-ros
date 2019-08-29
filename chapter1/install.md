# 第一节 安装配置ROS

ROS最好运行在Ubuntu系统下，每个版本的Ubuntu都对应不同的ROS，我用的是Ubuntu18.04，对应ROS  Memodic。下面说安装方法。参考[此处](http://wiki.ros.org/melodic/Installation/Ubuntu)。

---

### 1. 安装
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

ROS有多个组件，如qrt, rviz等，建议完整安装，即包含所有组件。完整安装命令为：

```
sudo apt install ros-melodic-desktop-full
```

会按"y"键然后回车确认安装。等一段时间，就会

### 2. 初始化rosdep

初始化rosdep有利于后续的编译和运行各种组件，所用命令为：

```bash
sudo rosdep init
rosdep update
```

### 3. 初始化环境变量

环境变量就是系统工作过程中的全局变量，ROS所在的库目录等都要告诉系统，为了避免每次运行ROS都要初始化一遍，可以把相关命令放在 ~/.bashrc 文件里，这样每打开终端时都会自动运行里面的脚本。方法为：

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

至此，就完成了ROS的安装和配置。请看下一节。