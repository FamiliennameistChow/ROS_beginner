# ROS_beginner
guide to learning ros

-----

# Table of Contents
* [installtion]()  
    * [如何正确安装ros_px4_gazebo仿真环境](https://github.com/FamiliennameistChow/ROS_beginner/blob/master/note/installtion/PX4_install.md)
    * [配置turtlebot_gazebo仿真环境](https://github.com/FamiliennameistChow/ROS_beginner/blob/master/note/installtion/turtlebot_gazebo_simulation.md)

    
* [ROS基础教程]()
    * [ROS wiki 实例(TO DO)](./note/ros_basic)


* [Pixhawk原生固件PX4开发指南(TO DO)]()   
    * [PX4 offboard Control with MAVROS(TO DO)]()

* [gazebo环境配置](https://github.com/FamiliennameistChow/ROS_beginner/blob/master/note/installtion/gazebo_simulation.md)
	* [gazebo中给无人机添加单目相机](https://zhuanlan.zhihu.com/p/91692124)
	* [gazebo中给无人机添加双目相机(TO DO)]
    * [gazebo_添加降落地标](https://github.com/FamiliennameistChow/ROS_beginner/blob/master/note/simulation/gazebo_add_landmark.md)

* [ROS功能包(可复用)](./packages)
	
* [AirSim环境配置(TO DO)]()

* [gazebo模型与场景](./gazebo/readme.md)

----

20191115 zb
* 添加"如何正确安装ros_px4_gazebo仿真环境"
* 添加"gazebo环境配置"

20191119 zb
* 添加"gazebo_添加降落地标"

20191121 zb
* 添加`vision/img_undistort.cpp`

20191124 zb
* 添加`配置turtlebot_gazebo仿真环境`

20191124 zb
* 添加`gazebo/mark_label_1, mark_label_h, mark_label_x`
* knn数字识别 添加 `vision/config/models.yml`
* 添加数字识别值　修改 `/vision/msg/detResult.msg`
* 添加标靶类型　修改`packages/vision/config/auto_landing_params.yaml`

20191125 zb

* 添加新的功能包`gazebo_test`用于控制gazebo模型状态

20191202 zwl
* 追踪降落增加位姿解算相关代码

---

`gazebo_test`功能包python 环境安装

* 创建虚拟环境

```
conda create -n DRL27 python=2.7
```
* 安装tensorflow等
```
pip install tensorflow==1.12.0
pip install opencv-python
pip install pillow
pip install pyyaml
```

* ImportError: No module named rospkg

```
pip install -U rosdep rosinstall_generator wstool rosinstall six vcstools
```
---

20191210 zb

* 增加着陆比赛图像检测代码`/vision/src/red_detect.cpp`
* 新增着陆比赛msg`/vision/msg/redResult.msg`

20191211 zwl
* 增加行星着陆比赛策略代码`/vision/src/planet_landing.cpp`

20191213 zwl
* 增加gazebo场景文件`/vision/world`，优化着陆策略。

20191224 zwl
* 增加库文件drone_flight_modes.hpp的功能，现在支持非控制模式和多机仿真。

20200312 zwl
* 增加将无人机下视深度图转为grid_map的代码，运行文件为 `grid_map_uav/sh/depth_to_gridmap.sh`。


20200321 zb
* 添加octomap重建demo -> `packages/test_octomap`
需要将`gazebo/worlds`下的`compeition.wold`添加到px4下的worlds下  
将`gazebo/models`下的`group_A`添加到px4下的models下 


20200323 zb
* 添加`navi_algorithm`导航算法包，目前二维rrt算法已实现

20200326 zb
* 添加`moon.world`月面场景，使用详见`gazebo/readme.md`

20200409 zb
* 更新`gazebo/models/Amy_terrain`模型
* 更新`test_octomap`添加无人机导航/turtlebot导航配置  
    * 查看`test_octomap`下的sh文件夹  
     1. pointcloud_to_octomap.sh用于无人机地面重建
     2. turtlebot_navi.sh用于turtlebot导航
     3. uav_navi.sh用于无人机导航

20200410 zwl
* 更新了无人机的下视点云转grid_map的代码，运行文件为 `grid_map_uav/sh/pointcloud_to_gridmap.sh`
* 新增支持：  
     1. 实时的点云转gridmap
     2. 全局地图拼接
     3. 使用滤波器生成表面法向量、坡度、粗糙度等信息层判断可通行区域，并根据可通行区域生成occupymap


20200411 zb
* 添加`packages/scout_ros`功能包  
　安装参考https://github.com/westonrobot/scout_ros/tree/melodic  

    ```sh
    sudo apt-get install ros-kinetic-gazebo-ros-control
    sudo apt install ros-kinetic-teleop-twist-keyboard
    sudo apt-get install ros-kinetic-joint-state-publisher-gui
    sudo apt install ros-kinetic-ros-controllers
    ```

* 使用请进入packages/scout_ros/navi_ros/sh
    * scout_navi.sh　使用四驱小车导航

20200414 zwl
* 更新了无人机的下视点云转grid_map的代码，运行文件为 `grid_map_uav/sh/pointcloud_to_gridmap.sh`
* 新增支持：  
     1. 自定义tf参数配置
     2. localmap和globalmap同时输出
     3. 实现整个过程的参数化配置

20200415 zwl
* 更新了octomap_scout包，增加了LOAM_velodyne为小车估计位姿

20200510 zb
* 更新test_octomap包，增加64线雷达，与costmap

20200520 zb
* 更新了octomap_scout包，增加aloam为小车估计位姿

20200520 zb
* 更新了octomap_scout包，增加基于aloam的导航方法
* 更新了octomap_scout包，更新octomap中二维地图抽取`src/process_2d_map.cpp`
* `octomap_scout/sh/scout_aloam_octomap.sh` 导航方法

20200610 zwl
* 更新了点云转gridmap的代码，将此功能独立出来作为一个服务包，供需要此功能的用户使用。

20200618 zb
* 更新`gazebo/models/Amy_terrain`模型,[参看](./gazebo)
* 更新了octomap_scout包，更新octomap中二维地图抽取`src/process_2d_map.cpp`，优化基于梯度的方案(推荐)
* 更新了octomap_scout包,增加了全局地图导入,[查看](./packages)

20200701 zb
* 更新了octomap_scout包，更新`src/process_2d_map.cpp`，添加局部地图

20200703 zwl
* 删除了gridmap_server，分成了gridmap_uav和gridmap_scout两个包。分别对无人机和无人车的应用场景做了优化。

20200726 zwl
* 新增moon_landing包，内容为月面着陆仿真代码．

20200901 zb
* 新增`octomap_navi`包，内容为小车基于八叉树的三维导航

20200902 zb
* 优化`octomap_navi`包，优化前端rrt路径查找显示代码


20200903 zb
* 更新了`octomap_scout`包，新增功能：
    1. 在aloam的基础上增加了回环检测
    2. 增加基于全局点云地图与栅格地图的初始定位
    3. 优化costmap更新策略，使用局部点云更新costmap
    4. 使用双目数据进行局部避障

* 注意本次更新*重构了文件目录*

20200903 zb  
* 更新了`octomap_navi`包，实现八叉树上的通行走廊生成

20200918 zb
* 更新`octomap_scout`包，新增功能:
    1. 提高位姿估计频率

20201008 zb
* 更新了`octomap_navi`包，实现mini-jerk轨迹优化

20201009 zwl
* 更新了`sjtu_game`包，存放交大无人机比赛代码。同时增加了gazebo文件夹中所需的models和world文件

20201010 
* 修改了gazebo文件夹中models文件里环的颜色

20201108 zb
* 更新了`sjtu_game`包，增加识别，控制，与路径规划

20201231 xhj
* 新增了`target_tracking`包,用于后续目标跟踪仿真实验