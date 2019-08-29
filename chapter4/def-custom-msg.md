# 第三节 自定义Messages

本节介绍怎样使用[Messages描述语言](http://wiki.ros.org/action/show/msg?action=show&redirect=ROS%2FMessage_Description_Language)自定义Messages 数据类型。此处只讲解C++实现，Python见[官方文档](http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages)。

### 1. 创建Messages

创建Messages很简单，直接把内容放在一个`.msg`文件中就OK。

### 2. Including Messages

在C++中，包含信息很简单，如下：

```c++
#include <std_msgs/String.h>

std_msgs::String msg;
```

### 3. 依赖

如果在不同的package中使用自定义的message，需要在`package.xml`文件中添加：

```xml
<build_depend>包含自定义msg的package名</build_depend>
<exec_depend>包含自定义msg的package名</exec_depend>
```

在`CMakeLists.txt`文件中也需要添加：

```cmake
add_dependencies(你的当前program ${catkin_EXPORTED_TARGETS})
```

