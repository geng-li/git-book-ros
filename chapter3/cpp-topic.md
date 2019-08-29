# 第一节 C++实现Topics通信

之前提到，Topics通信是两个节点一个发消息，一个接收消息。下面来到最美妙的时刻，即用C++编程实现。

### 1. 准备

进入随我们一路走来的 beginner_tutorials package，然后新建一个文件夹，叫 src，通常我们把C++代码放在这个文件夹里。

```bash
roscd beginner_tutorials #报错的话看看有没有把 ~/catkin_ws/devel/setup.bash 加到 ~/.bashrc 文件最后一行
mkdir -p src
```

在开始写代码之前，我们想想需要哪些东西。Topics通信，我们假设有两个 Nodes，一个只管发送，一个只管接收。

对于发送方，应该有个发送方的对象，还要有发送的信息，然后对象调用发送信息这个方法来实现发信息。发送的Topics应该有个独一无二的名字，在代码实现的时候，可能要初始化一下环境等等。

对于接受方，类似。我们实现的时候，发送方Node是一个文件，接收方Node是另一个文件。两者完全独立。

下面看看代码怎样实现。

### 2. Publisher

新写一个文件，名为 talker.cpp，保存在上一步建的 src/ 文件下（**不是** catkin_ws/ 下面那个 src/）。

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    // 初始化，第三个参数是 Node 的名字
    ros::init(argc, argv, "talker");
    
    // 创建NodeHandle，NodeHandle就像一个Node指针，拥有Node的全部方法和属性
    ros::NodeHandle n;
    
    /**
    /* advertise()用来指定消息怎样发送，调用后，ROS Master会就注册一个发布节点，如果有其他接受者需要这个节点的信息，ROS Master会让它们建立起联系。这个函数返回一个 Publish 对象，后面可以直接调用发布信息。 
    第一个参数是 Topic 的名字，第二个是ROS处理不过来太多消息时，消息最多能暂存多少条。
   */
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    
    // 我们想一直发送消息
    int count = 0; //计数发送的消息数
    ros::Rate loop_rate(10); //消息发送频率
    while (ros::ok())
    {
        // 下面指定发送的消息
        std_msgs::String msg;
        msg.data = "A nice try!!!";

        // 发送消息
        chatter_pub.publish(msg);
        // 把发送的消息在终端显示，可选
        ROS_INFO("%s%d%s", "Publist -- ", count, msg.data.c_str());
        loop_rate.sleep();
        ++count;
    }
    
    
    return 0;
}
```

*注* : [官方代码](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) 里面还多了个 ros::spinOnce() ，我感觉对于 Publisher 来说不需要。详见[此处](https://www.cnblogs.com/liu-fa/p/5925381.html)。

### 3. Subcriber

创建 listener.cpp ，放在和 takler.cpp 同一个文件夹下。

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

// 向控制台输出指定信息
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I hear: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    // 注意第一个参数 chatter 和上面Publisher的参数一致，同一个Topic
    // 第三个参数表示接受到消息后做的事
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    
    // 上面只是主程序定义，要跳到后台执行ROS处理，需要下面这个函数，之后上面的sub会一直处于接收状态，程序也不会停止，最后的 return 0 其实完全没有意义。
    ros::spin();
    
    return 0;
}
```

### 4. 编译运行

增添了新的C++文件，肯定要修改CmakeLists.txt文件（是 beginner_tutorials/ 下面那一个），告诉编译器要编译这些文件。直接在 CMakeLists.txt 最后添加如下代码就好：

```cmake
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```

最后，整个 CMakeLists.txt 文件看起来如下。

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```

之后，就进行编译：

```bash
cd ~/catkin_ws
catkin_make
```

没报红色的 Err 就是成功了。

### 5. 总结

以上便是一个Publisher和Subscriber的例子。下面我们看看怎么调用编译好的 Publisher 和 Subscriber。

拭目以待！