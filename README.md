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
* 增加将无人机下视深度图转为grid_map的代码。


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
