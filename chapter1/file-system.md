

# 第二节

这一节介绍ROS的一些文件管理工具。ROS里面有很多应用，比如一个机器人，有摄像头、有激光传感器、有运动控制，那么我们可以模块化地进行相关控制。每个模块放在一个文件夹里，这样的一个文件夹就是一个package。我们先不介绍怎么写packages，先看看怎么使用一些命令来方便查看pacakges。具体参考[此处](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)。

---

### 1. rospack

rospack可以得到ROS中packages的信息。rospack可以加不同的选项，这里以rospack find为例：

```bash
rospack find [package_name]
```

比如：

```
rospack find roscpp
```

就是查找roscpp这个package的路径。我的运行提示

```bash
Command 'rospack' not found, but can be installed with:

sudo apt install rospack-tools
```

就照个提示，运行

```
sudo apt install rospack-tools
```

之后再运行```rospack find roscpp```，一切正常，输出：

```
/opt/ros/kinetic/share/roscpp
```



### 2. roscd

比如上面查找到了roscpp，那怎样快速进入roscpp所在文件夹呢？就用roscd，例如

```bash
roscd roscpp
```

再用

```bash
pwd
```

查看当前路径。可以发现输出和上面的```rospack find```的结果一样。

```bash
roscd log
```

可以查看ROS的输出日志，在报错的时候有用。注：初次时没有运行过ROS时可能没有相关日志文件。

### 3. rosls

rosls 可以帮我们快速看某个包里面的文件结构，例如：

```bash
rosls rospy
```

可以查看rospy这个package里面的文件。

### 4. tab键

tab键很好用，可以自动补全很多ROS命令及其选项，如果超过一种可能补全方法，两次tab显示相关可能选项。

### 5. 总结

* rospack = ros + pack(age)
* roscd = ros + cd
* rosls = ros + ls

下一节讲解创建package。