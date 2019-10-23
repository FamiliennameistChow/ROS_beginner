# gazebo仿真

----

[参考](https://blog.csdn.net/oqqENvY12/article/details/55263122)
[gazebo仿真px4](https://fantasyjxf.gitbooks.io/px4-wiki/content/4_Simulation/gazebo_simulation.html)

----

* 假设你已经正确安装了ros gazebo与px4 toolchain

[参考这里](./如何正确安装ros_px4_gazebo仿真环境.md)

[参考](https://blog.csdn.net/weixin_41922934/article/details/80601613)
```
dpkg -l | grep gazebo
```

## 外部控制

* 外部启动gazebo 仿真器
[gazebo仿真px4](https://fantasyjxf.gitbooks.io/px4-wiki/content/4_Simulation/gazebo_simulation.html)
1. 四旋翼
```
cd {~/path_to_Firmware}
make posix_sitl_default gazebo
```

2. 四旋翼带光流模块 【error】
```
cd {~/path_to_Firmware}
make posix gazebo_iris_opt_flow
```



* 启动`mavros`
mavlink会开放`14557`端口，将MAVROS连接到这个端口将会接收到实际飞行中发出的所有数据  
如果需要ROS接口，那么已经在运行的次级(secondary)MAVLink实例可以通过mavros连接到ROS。若要连接到一个特定的IP(fcu_url是SITL的IP/端口)，请使用一个如下形式的URL:

```
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```

连接本地:
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

* 仿真控制

## 使用ros装饰器启动gazebo(Launching Gazebo with ROS Wrappers)

[中文wiki](https://fantasyjxf.gitbooks.io/px4-wiki/content/4_Simulation/interfacingto_ros.html)
[英文详解](http://dev.px4.io/v1.9.0/en/simulation/ros_interface.html)


-----

如果想要修改Gazebo仿真，使其能够将额外的传感器信息直接发布到ROS主题，例如Gazebo ROS激光传感器信息，那么必须通过适当的ROS包装器来启动Gazebo

官方提供的两个launch文件
>posix_sitl.launch: plain SITL launch
>mavros_posix_sitl.launch: SITL and MAVROS

* 在~/.bsahrc中添加

```
source {path_to_Firmware}/Tools/setup_gazebo.bash {path_to_Firmware} {path_to_Firmware}/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:{path_to_Firmware}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:{path_to_Firmware}/Tools/sitl_gazebo
```

打开新的终端，执行
`roslaunch px4 posix_sitl.launch`


