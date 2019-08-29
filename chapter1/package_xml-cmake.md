# 第四节 package.xml和CMakeLists.txt

这一节介绍```package.xml```和```CMakeLists.txt```，前面提到，这两个文件是一个package所必须的。

### 1. package.xml

package.xml更多信息详见[这里](http://wiki.ros.org/catkin/package.xml)。用编辑器打开文件，我们可以看到如下内容，汉语部分的注释是我添加的。

```xml
<?xml version="1.0"?>
<package format="2">
  <name>beginner_tutorials</name>
  <version>0.0.0</version>
  <description>The beginner_tutorials package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="gengli@todo.todo">gengli</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>

    
  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/beginner_tutorials</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```

前面的部分，如下：

```xml
  <name>beginner_tutorials</name>
  <version>0.0.0</version>
  <description>The beginner_tutorials package</description>
  <maintainer email="gengli@todo.todo">gengli</maintainer>
  <license>TODO</license>
```

描述了文件的名字、版本、描述、作者、版权等信息，不算重要。

后面的

```xml
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
```

描述了build时的依赖，可以看出这些依赖和用```catkin_create_pkg```时指定的依赖是一致的。

之后的

```xml
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

为了让所有指定的依赖在构建（build）和运行（exec）的时候都能正常获取到，我们还指定了<exec_depend>标签。

### 2. CMakeLists.txt

该文件更详细内容见[官方文档](http://wiki.ros.org/catkin/CMakeLists.txt)。ROS采用略微改动的cmake编译方式，如果你了解过cmake，一定对这个文件很熟悉。简单来说这个文件指定了C++（也可以是Python）的编译方式，在命令行运行一个C++文件还不算难，但要运行多个依赖，还要链接各种库的文件，用单纯的命令行来写太麻烦了，那就把链接关系等写入一个文件，这就是CMakeLists.txt文件。用下面命令可以查看我们的例子中的CMakeLists.txt

```
cd ~/catkin_ws/src
gedit beginner_tutorials/CMakeLists.txt
```

去掉注释后的内容如下：

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
```

如果你英语不错的话，看一遍来面的注释就知道该怎么设置了（我上面删了大部分注释掉的内容），具体的我们遇见了再细说。对于初学者，不用掌握那么多，只要能根据模板简单修改得到自己想要的内容就好了。

### 2. 总结

这一节主要讲了 package.xml 和 CMakeLists.txt 文件，对于初学者，也行觉得里面很复杂，多了解一点其他内容，再回过头来，就会觉得容易接受得多。上面的东西没有完全弄懂关系也不大，现在没有练习，光介绍概念只会越介绍越混乱。

下一节介绍构建或者说编译（build）我们的文件。

