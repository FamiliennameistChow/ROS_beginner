# 拓噗(TuoPu->tp)项目(tp_project)的工程文件

## car_sim_description 

包含了两个车型， 全向麦轮车(holonomic) 和 舵轮车（turtlebot3）

在 launch 文件中 根据搭配的传感器是 一套还是 两套 分为 single 和 double 两种

## realsense_ros_gazebo 

相机的模型文件

## tp_slam

1、调用了 cartographer SLAM 功能包，需要自己配置： [配置链接](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)

## turtlebot3

里面主要包含了，turtlebot的 控制接口，还有 move_base 的配置

## aws-robomaker-hospital-world
医院的模型和世界环境

## car_junction
交通道路的模型和世界环境

## 使用方法

### 依赖安装

1、 配置 cartographer： [安装方法](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)

2、 安装 move-base

```
 sudo apt install ros-noetic-move-base
```
3、 建图
```
// 开启 仿真场景 (全向轮类似的打开场景文件)
roslaunch car_sim_description turtlebot3_single_sensor.launch
// 或者
roslaunch car_sim_description turtlebot3_double_sensor.launch
// 打开医院仿真环境 找到模型文件位置
export GAZEBO_MODEL_PATH=`pwd`/models:`pwd`/fuel_models
roslaunch car_sim_description turtlebot3_single_sensor_hospital.launch

// 开启 SLAM 
roslaunch tp_slam single_sensor_slam.launch
// 或者
roslaunch tp_slam double_sensor_slam.launch

// 开始 move_base
roslaunch turtlebot3_navigation move_base.launch
```
4、 地图保存, 可以先利用下面命令保存.pbstream格式地图 (此处已有地图,这一步不需要)
```
rosservice call /finish_trajectory 0

rosservice call /write_state "{filename: '${HOME}/Downloads/b3-2016-04-05-14-14-00.bag.pbstream', include_unfinished_submaps: "true"}"
```
5、 重定位
[参考连接](https://www.freesion.com/article/9794248452/)

打开cartographer源码, 修改文件occupancy_node_main.cc, 这个文件在路径cartographer_ros-master\cartographer_ros\cartographer_ros\occupancy_grid_node_main.cc

将该文件替换成tool_packages文件夹下的[occupancy_grid_node_main.cc](./../../tool_packages/cartographer_ros/cartographer_ros/cartographer_ros/occupancy_grid_node_main.cc)文件,并重新编译

启动重定位, 此处为已经保存好的医院地图文件, 打开医院gazebo仿真环境, 执行下面命令
```
roslaunch tp_slam single_sensor_slam_localization.launch
```

其中 开启 仿真场景和 开启SLAM 需要对应开启。 开启SLAM 之后可以稍微等几秒钟，雷达默认的更新频率是 5 Hz，SLAM 系统需要一定量的数据输入，大概为20帧左右。

* [ ] cartographer 多传感器的机制有一些问题，所以双雷达，效果不一定比 单雷达好。正在改进中。。。。
