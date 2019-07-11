---
title: ROS_Node_Communication_II
published: true
---

### [](#header-3) Concepts

msg文件就是一个简单的txt文件，其中每行有一个类型和名称，可用的类型如下：

```txt
int8, int16, int32, int64 (plus uint*)
float32, float64
string
time, duration
other msg files
variable-length array[] and fixed-length array[C] 
Header(包含一个timestamp和坐标系信息)
```

srv文件和msg文件很相像，除了它包含两个部分：请求和回应。

```txt
int64 A
int64 B
---
int64 sum
```

上面一部分为request，下面一部分为response。

**I.**

1. 创建msg

首先，在 chapter2_tutorials 功能包下创建 msg 文件夹，并在其中创建一个新的文件 chapter2_msg1.msg。

```bash
$ cd ~/dev/catkin_ws/src/chapter2_tutorials
$ mkdir msg
$ vim chapter2_msg1.msg
```

并在文件中增加如下行：

```txt
int32 A
int32 B
int32 C
```

现在编辑 package.xml，从 message_generation 和 message_runtime行删除<!-- -->，按照下面加入 message_generation：

```xml
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
```



找到如下行，取消注释，并加入新消息名称：

```xml
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  chapter2_msg1.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

现在，用下面命令进行编译：

```bash
$ cd ~/dev/catkin_ws/
$ catkin_make
# 检查编译是否成功，使用下面 rosmsg 命令, 如果你在 chapter2_msg1 文件中看到一样的内容，说明编译正确。
$ rosmsg show chapter2_tutorials/chapter2_msg1
```



2. 创建srv文件

然后创建一个 srv 文件。在 chapter2_tutorials 文件夹下创建一个名为 srv 的文件夹，并新建文件 chapter2_srv1.srv，在文件中增加以下行：

```txt
int32 A
int32 B
int32 C
---
int32 sum
```

为了编译新的 msg 和 srv 文件，必须取消在 package.xml 和 Cmakefile.txt 中的如下行的注释。这些包括消息和服务的配置信息，并告诉ROS如何编译。

```bash
# 首先，按下面方式从 chapter2_tutorials 功能包中打开 CMakefile.txt 文件：
$ rosed chapter2_tutorials CMakefile.txt
```

找到下面行，取消注释，并改为正确数据：

catkin_package(

  CATKIN_DEPENDS message_runtime

)

在 add_service_files 如下位置添加服务文件的名字：

add_service_files(

  FILES

  chapter2_srv1.srv

)

```bash
# 现在，用下面命令进行编译：
$ cd ~/dev/catkin_ws/
$ catkin_make
#测试编译是否成功，使用如下 rossrv 命令, 如果你看到跟 chapter2_srv1 文件中相同的内容，说明编译正确。
$ rossrv show chapter2_tutorials/chapter2_srv1
```

```c++
// 消息发布节点
#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_srv1.h" //包含先前所创建的srv文件

//对3个变量求和，并将计算结果发生给其他节点，Request为上一部分的请求，而Response为下一部分的响应
bool add(chapter2_tutorials::chapter2_srv1::Request  &req,
         chapter2_tutorials::chapter2_srv1::Response &res)
{
  res.sum = req.A + req.B + req.C;
  ROS_INFO("request: A=%d, B=%d C=%d", (int)req.A, (int)req.B, (int)req.C);
  ROS_INFO("sending back response: [%d]", (int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_3_ints_server");
  ros::NodeHandle n;

  //创建服务并在ROS中发布广播
  ros::ServiceServer service = n.advertiseService("add_3_ints", add);
    
  ROS_INFO("Ready to add 3 ints."); //在命令行窗口输出信息
  ros::spin();

  return 0;
}
```

```c++
// 消息接受节点
#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_srv1.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_3_ints_client");
  if (argc != 4)
  {
    ROS_INFO("usage: add_3_ints_client A B C ");
    return 1;
  }
  ros::NodeHandle n;
  //以add_3_ints为名称创建一个服务的客户端
  ros::ServiceClient client = n.serviceClient<chapter2_tutorials::chapter2_srv1>("add_3_ints");
  //下面创建srv文件的一个实例，并且加入需要发生的数据值
  chapter2_tutorials::chapter2_srv1 srv;
  srv.request.A = atoll(argv[1]);
  srv.request.B = atoll(argv[2]);
  srv.request.C = atoll(argv[3]);
 
  //调用服务并发生数据。如果调用成功，call()函数会返回true；如果没成功，call()函数会返回false
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
  return 0;
}
```



为了编译节点，在 CMakefile.txt 文件中增加如下行：

add_executable(chap2_example2_a src/example2_a.cpp)

add_executable(chap2_example2_b src/example2_b.cpp)

add_dependencies(chap2_example2_a chapter2_tutorials_generate_messages_cpp)

add_dependencies(chap2_example2_b chapter2_tutorials_generate_messages_cpp)

target_link_libraries(chap2_example2_a ${catkin_LIBRARIES})

target_link_libraries(chap2_example2_b ${catkin_LIBRARIES})

```bash
# 现在执行以下命令：
$ cd ~/dev/catkin_ws
$ catkin_make
# 为了启动节点，需要执行以下命令行：
$ rosrun chapter2_tutorials chap2_example2_a
$ rosrun chapter2_tutorials chap2_example2_b 1 2 3
```

 

**II.**



```c++
#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_msg1.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example3_a");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<chapter2_tutorials::chapter2_msg1>("message", 1000);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    //这里使用了自定义消息类型int32 A，int32 B,int32 C
    chapter2_tutorials::chapter2_msg1 msg;
    msg.A = 1;
    msg.B = 2;
    msg.C = 3;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
```



 ```c++
#include "ros/ros.h"
#include "chapter2_tutorials/chapter2_msg1.h"

void messageCallback(const chapter2_tutorials::chapter2_msg1::ConstPtr& msg)
{
  //这里使用了自定义消息类型int32 A，int32 B,int32 C
  ROS_INFO("I heard: [%d] [%d] [%d]", msg->A, msg->B, msg->C);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example3_b");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("message", 1000, messageCallback);
  ros::spin();
  return 0;
}
 ```

 

命令行窗口1执行 roscore

新建窗口2执行 rosrun chapter2_tutorials chap2_example2_a

新建窗口3执行 rosrun chapter2_tutorials chap2_example2_b

 

命令行窗口1执行 roscore

新建窗口2执行 rosrun chapter2_tutorials chap2_example3_a

新建窗口3执行 rosrun chapter2_tutorials chap2_example3_b

新建窗口4执行 rqt_graph

![Screen Shot 2019-07-10 at 9.39.23 AM.png](https://i.loli.net/2019/07/11/5d26dd7e4dba667276.png)![Screen Shot 2019-07-10 at 9.39.34 AM.png](https://i.loli.net/2019/07/11/5d26dd9c1dfa740271.png)

![Screen Shot 2019-07-10 at 9.46.25 AM.png](https://i.loli.net/2019/07/11/5d26e4354860466169.png) ![Screen Shot 2019-07-10 at 9.46.03 AM.png](https://i.loli.net/2019/07/11/5d26e4e3e6ce042673.png)

 ![Screen Shot 2019-07-11 at 10.45.19 AM.png](https://i.loli.net/2019/07/11/5d26e5080436f19795.png)