- [cn](#操作指南)
- [en](#Instructions)
# 操作指南

>此SDK仅适用于以下产品型号的激光雷达产品:
>- LDROBOT LiDAR STP23
>- LDROBOT LiDAR STP23L

## 1. 系统设置
- 将雷达连接到你的系统主板，设置雷达在系统中挂载的串口设备-x权限(以/dev/ttyUSB0为例)
	- 实际使用时，根据雷达在你的系统中的实际挂载情况来设置，可以使用`ls -l /dev`命令查看.

``` bash
sudo chmod 777 /dev/ttyUSB0
```

  - 修改`ros2_app/src/ldlidar/launch/ (产品型号).launch.py`文件中的port_name值，以`stp23.launch.py`和`/dev/ttyUSB0`为例，如下所示.

```py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar',
      executable='ldlidar',
      name='STP23',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_STP23'},
        {'topic_name': 'laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 921600},
        {'frame_id': 'base_laser'}
      ]
  )

  # base_link to base_laser tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_stp23',
    arguments=['0','0','0.18','0','0','0','base_link','base_laser']
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)

  return ld
```
## 2. 编译方法

使用colcon编译，在readme文件所在目录下执行如下指令.

```bash
colcon build
```
## 3. 运行方法

```bash
source install/local_setup.bash
```
- 产品型号为 LDROBOT STP23

  ``` bash
  ros2 launch ldlidar stp23.launch.py  # start node 

  ros2 launch ldlidar viewer_stp23.launch.py # start node and show data on the Rviz2
  ```

- 产品型号为 LDROBOT STP23L

  ``` bash
  ros2 launch ldlidar stp23l.launch.py  # start node 

  ros2 launch ldlidar viewer_stp23l.launch.py # start node and show data on the Rviz2
  ```

##   4. 可视化

> 代码支持ubuntu20.04 ROS2 foxy版本及以上测试，使用rviz2可视化。
- 新打开一个终端 (Ctrl + Alt + T),并通过Rviz2工具打开rviz2文件夹下面的rviz配置文件
```bash
rviz2
```
![rviz2_demo](../doc/stp_rviz.png)

[回到仓库简介](../README.md)

# Instructions

> This SDK is only available for lidar products of the following product models：
> - LDROBOT LiDAR STP23
> - LDROBOT LiDAR STP23L

## step 1: system setup
- Set the permission of serial port device mounted by LiDAR in the system(example:device name is /dev/ttyUSB0)
    - The actual use of the radar is based on the actual mounted on your system, you can use the `ls -l /dev` command to view. 

``` bash
sudo chmod 777 /dev/ttyUSB0
```
  -  Modify port_name value in the `ros2_app/src/ldlidar/launch/`(product name).launch  files,

   > for example `stp23.launch.py` and `/dev/ttyUSB0`.

``` py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar',
      executable='ldlidar',
      name='STP23',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_STP23'},
        {'topic_name': 'laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 921600},
        {'frame_id': 'base_laser'}
      ]
  )

  # base_link to base_laser tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_stp23',
    arguments=['0','0','0.18','0','0','0','base_link','base_laser']
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)

  return ld
```
## step 2: build

Run the following command in the directory where the readme file resides.

```bash
colcon build
```
## step 3: run

```bash
source install/local_setup.bash
```
- The product is LDROBOT STP23
  ``` bash
  ros2 launch ldlidar stp23.launch.py  # start node 

  ros2 launch ldlidar viewer_stp23.launch.py # start node and show data on the Rviz2
  ```

- The product is LDROBOT STP23L
  ``` bash
  ros2 launch ldlidar stp23l.launch.py  # start node 

  ros2 launch ldlidar viewer_stp23l.launch.py # start node and show data on the Rviz2
  ```

## step 3: visuallization

> The code supports ubuntu 20.04 ros2 foxy version and above, using rviz2 visualization.

- new a terminal (Ctrl + Alt + T) and use Rviz2 tool ,open the rviz config file below the rviz2 folder
```bash
rviz2
```
![rviz2_demo](../doc/stp_rviz.png)

[ Back to the introduction ](../README.md)

