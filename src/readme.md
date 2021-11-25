# 《无人驾驶系统与控制》 实验代码示例

## 1. 基本技能要求
* Linux系统常用指令 
* **ROS 基本操作(重要)** 可参考[官网教程](http://wiki.ros.org/)或其他网络博客
* 编程语言 C++ (推荐) 或者 Python
* 基本图像处理与点云处理操作

## 2. 小车调试方式

### 2.1 使用无线键鼠与车载显示屏

可以在现场调试时临时使用，不建议直接在小车上修改代码。

### 2.2 使用ssh方式远程登录小车的`ubuntu`系统

这种操作适用于`ubuntu`, `macOS`和`windows`系统。

* a. 将自己的电脑与小车同时连接在局域网下，如手机热点等。
* b. 在小车终端中输入如下命令，查询小车的ip地址
``` bash
$ ifconfig
```
* c. 打开`ubuntu`或`macOS`系统的终端，`windows`系统需要使用`putty`软件，通过ssh远程登录小车。
``` bash
#  xxx.xxx.xxx.xxx为小车局域网ip地址
$ ssh pi@xxx.xxx.xxx.xxx
# 输入密码 pi
# 登录成功
``` 
详细操作还可参考大量网络博客。

### 2.3 使用`git`和`github`上的远程仓库

按照[参考博客](https://blog.csdn.net/zamamiro/article/details/70172900?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-2.control&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-2.control)完成初次上传。

在调试使用时，可将通过如下命令本地代码传到云端：
```
$ git add .
$ git commit -m "ANY STRING YOU WANT"
$ git push -u origin master
```
在小车上通过如下命令同步云端代码：
```
$ git pull origin master
```
也可将小车代码上传云端，本地同步。

### 2.4 使用`Teamviewer` 或者 `VNCserver`等远程桌面
自行探索。

## 3. 代码使用说明
* `bluetooth_bridge`: 蓝牙以及串口通讯驱动代码，其中`serial_port_server.py`是将控制指令传递给底层驱动的文件，重要但无需花费精力理解。
* `control`: 舵机控制。
* `lane_detection`: 视觉巡线代码示例。
* `obstacle_detection`: 激光雷达停障示例。
* `rplidar_ros`: 激光雷达驱动。
* `teleop_twist_keyboard`: 键盘控制。

``` bash
$ cd ~
# 下载代码
$ git clone https://github.com/Chow76/autonomous_driving_Class.git
$ cd huaweicar_ws
# 编译
$ catkin_make
$ source devel/setup.bash
# 需要修改python文件的权限 仅对第一次建立的python文件执行
$ sudo chmod +x ./src/lane_detection/scripts/lane_detection.py
$ sudo chmod +x ./src/obstacle_detection/scripts/obstacle_detection.py
$ sudo chmod +x ./src/control/scripts/servo_node.py
$ sudo chmod +x ./src/bluetooth_bridge/scripts/serial_port_serve_node.py

# 测试巡线代码
$ roslaunch lane_detection lane_test.launch
# 测试停障代码
$ roslaunch obstacle_detection obstacle_test.launch
```

**重要：** 示例代码中的巡线算法和停障算法只是从传感器获取数据到车辆控制的流程示例，效果可能不够理想。建议自行选择算法处理传感器数据实现功能。

## 4. 备注

如有疑问请联系助教，祝大家有所收获！