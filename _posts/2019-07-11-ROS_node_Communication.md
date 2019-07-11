---
title: Sample Page
published: true
---

# [](#header-1) ROS

### ### [](#header-3) Concepts

工作空间是一个包含功能包、可编辑源文件或者编译包的文件夹。

源空间（The Source space）：在源空间（src文件夹）

编译空间（The Build space）:在build文件夹里；

开发空间（The Development (devel) space ）:devel文件夹用来保存 编译后的程序；

功能包（Package）：是ROS中软件组织的基本形式。一个功能包具有用于创建ROS程序的最小结构和最少内容。包含ROS运行的进程（节点）、配置文件等。

消息类型（Message  (msg)  type ）:消息是一个进程发送到其它进程的信息。ROS系统有很多标准类型消息。消息类型的说明存储在my_package/msg/MyMessageType.msg中，即对应功能包的msg文件夹下。

服务类型（Service (srv) type）:服务描述说明存储在my_package/msg/MyMessageType.srv中；定义了在ROS中每个进程提供的关于服务请求和响应的数据结构.

 主题（Topic）:是节点间用来传输数据的总线

### ### [](#header-3) 步骤

1. 创建工作空间

```bash
mkdir –p ~/catkin_ws/src
# 其中catkin_ws 为实例工作空间的名字
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
# 配置环境变量
source ~/catkin_ws/devel/setup.bash
```



2. 创建功能包

```bash
cd ~/catkin_ws/src
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
ros_demo_pkg   roscpp std_msgs actionlib actionlib_msgs
```



3. 创建节点

发布者节点:

```c++
//roscpp client APIs的核心头文件
#include "ros/ros.h"
//标准消息类型Int32的头文件
#include "std_msgs/String.h"
#include <sstream>

// argc是命令行总的参数个数
// argv[]是argc个参数，其中第0个参数是程序的全名，以后的参数命令行后面跟的用户输入的参数，
// ------------------------
// char *argv[]是一个字符数组,其大小是int argc,
//      主要用于命令行参数argv[]参数，数组里每个元素代表一个参数;

int main(int argc, char **argv)
{
   ros::init(argc , argv , "example1_a");
   //创建一个节点句柄对象，用于和ROS系统通讯
   ros::NodeHandle n; 
   //创建一个主题发布者对象，设定主题名、消息类型和缓冲区大小
   ros::Publisher chatter_pub= n.advertise<std_msgs::String>("message",1000);
   //设定发送数据的频率
   ros::Rate loop_rate(10);
   while (ros::ok())
   {
      //创建 String 型消息对象
      std_msgs::String msg;
      std::stringstream ss;
      ss << "I am the example1_a_node ";
      msg.data=ss.str();

      //将消息发布到主题
      chatter_pub.publish(msg);
      //读取并更新所有的topics
      ros::spinOnce();
      //实现数据发布频率
      loop_rate.sleep();
   }
   return 0;
}
```

订阅者节点:

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

//Callback 函数，当有数据被发布到主题/numbers时会调用该函数
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO(" I heard : [%s]" ,msg->data.c_str());
}


int main(int argc , char **argv)
{
  //初始化ROS node
  ros::init(argc, argv, "example1_b");
  ros::NodeHandle n;
  //创建一个订阅者对象，设定主题名、缓冲区大小和callback函数
  ros::Subscriber sub= n.subscribe("message", 1000,chatterCallback);
  ros::spin();
  return 0;
}
```

4. 构建节点

 在已有的CMakeLists.txt 文件中加入以下代码

```bash
# 指定头文件位置
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(chap2_example1_a src/example1_a.cpp)
add_executable(chap2_example1_b src/example1_b.cpp)

# 添加依赖
add_dependencies(chap2_example1_a chapter2_tutorials_generate_messages_cpp)
add_dependencies(chap2_example1_b chapter2_tutorials_generate_messages_cpp)

# 添加链接库
target_link_libraries(chap2_example1_a ${catkin_LIBRARIES})
target_link_libraries(chap2_example1_b ${catkin_LIBRARIES})
```

然后在命令行运行

```bash
cd ~/catkin_ws
catkin_make

# 命令行窗口1执行 
roscore
# 新建窗口2执行 
rosrun chapter2_tutorials chap2_example1_a
# 新建窗口3执行 
rosrun chapter2_tutorials chap2_example1_b
# 新建窗口4执行 
rqt_graph
```



![../Desktop/Screen%20Shot%202019-07-11%20at%2010.37.43%20AM.png](file:////Users/junchengzhu/Library/Group%20Containers/UBF8T346G9.Office/msoclip1/01/clip_image002.gif)

![../Desktop/Screen%20Shot%202019-07-11%20at%2010.37.25%20AM.png](file:////Users/junchengzhu/Library/Group%20Containers/UBF8T346G9.Office/msoclip1/01/clip_image004.gif)



![../Desktop/Screen%20Shot%202019-07-11%20at%2010.38.38%20AM.png](file:////Users/junchengzhu/Library/Group%20Containers/UBF8T346G9.Office/msoclip1/01/clip_image006.gif)

使用rqt_graph命令能够创建一个显示当前系统运行情况的动态图形，如图所示。

example1_a 节点发布 /message 主题，同时 example1_b 节点订阅了这个主题。