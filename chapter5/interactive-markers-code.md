# 第五节 代码实现简单交互形状服务

这一节，我们来实现一个简单的交互功能。参考[官方文档](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Writing%20a%20Simple%20Interactive%20Marker%20Server) 。

### 1. 简介

这个例子也是ROS自带的，我们可以参照上一节的做法，运行

```bash
rosrun interactive_marketutorials simple_marker
```

效果如下：

![](../images/rviz_interactive_simple_marker.png)



### 2. 实现

#### 2.1 源码

源代码可以从[这里](https://raw.githubusercontent.com/ros-visualization/visualization_tutorials/indigo-devel/interactive_marker_tutorials/src/simple_marker.cpp) 下载。

```c++
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

// 调用此函数时在终端输出形状所在的坐标
void processFeedback(
	const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM( feedback->marker_name << " 现在在 " << feedback->pose.position.x << "，" << feedback->pose.position.y << "，" << feedback->pose.position.z );
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_marker");
    
    // 在 simple_marker 话题空间创建一个交互式的形状服务
    interactive_markers::InteractiveMarkerServer server("simple_marker");
    
    // 给服务器创建交互形状
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = "my_marker";
    int_marker.description = "简单的 1-DOF 控制";

    // 创建一个灰色的立方体
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.45;
    box_marker.scale.y = 0.45;
    box_marker.scale.z = 0.45;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;

    // 创建一个包含立方体的非交互性的控制
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( box_marker );
    
    // 把控制加到交互形状上
    int_marker.controls.push_back( box_control );
    
    // 创建一个没有包含任何形状，可以移动立方体的的控制
    // 这会让rviz插入两个箭头
    visualization_msgs::InteractiveMarkerControl rotate_control;
    rotate_control.name = "move_x";
    rotate_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    
    // 把控制增加到交互形状上
    int_marker.controls.push_back(rotate_control);
	
	// 把交互形状加到我们的管理系统
    // 收到反馈信息的时候让服务器调用 processFeedback()
    server.insert(int_marker, &processFeedback);
    
    // 触发改变并把改变发送给客户端
    server.applyChanges();
    
    // 让ROS不断循环
    ros::spin();
}
```

#### 2.2 步骤

以上代码我们主要做了以下事情：

- 定义了一个*processFeedback* 函数，通过向控制台输出位置的方式来处理来自rviz的反馈信息；
- 初始化 *roscpp* ；
- 创建一个交互式的形状服务对象；
- 开始一个交互形状，并把它添加到服务的收集站；
- 进入ROS message循环。

注意，当我们调用 *insert* 函数的时候，服务对象会把形状放到一个等待列表，一旦调用 *applyChanges* 是，它会把形状放入交互形状的公开可见的集合里，并把交互的形状发送给和它连接的所有客户端。



































