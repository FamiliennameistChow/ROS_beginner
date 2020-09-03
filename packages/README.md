# ROS_beginner
包含了适用于UAV和UGV的多个ROS包，可以实现不同的功能

-----

# Table of Contents   
## tool packages for users    

## packages for UAV 
* [drone_lora包](./UAV/drone_lora)  
此功能包为在无人机的机载计算机上使用串口和socket的包。src文件夹中为使用serial和socket的示例代码。

* [px4_mavros包](./UAV/px4_mavros)  
此功能包为基础学习和无人机控制包。包括基本的offboard模式控制无人机以及扩展而来的键盘控制无人机（共三个版本）。include文件夹内包含一个库文件。将此库文件作为头文件，便可以使用类AeroDrone和其中的成员函数控制无人机或获取无人机参数。src文件夹内drone_flight_modes_node.cpp为库文件使用方法示例代码。

* [vision包](./UAV/vision)  
此功能包为无人机视觉相关包。 
    ```
    img_pub.cpp为打开USB相机并发布ROS图像消息；img_sub为接收ROS图像消息并显示   
    auto_landing为静态地标降落相关代码； mark_tracking为追踪降落相关代码    
    detect_mark为数字或H型地标识别相关代码    
    red_detect为红色标靶识别代码，与planet_landing一起组成自主着陆比赛的功能代码。
    ```   
    运行程序请运行sh文件夹中相应的.sh文件.

* [gridmap_uav包](./UAV/gridmap_uav)  
此功能包为无人机下视传感器数据转gridmap的功能包。depth_to_gridmap.hpp为无人机下视深度图转gridmap的功能包。pointcloud_to_gridmap.hpp为下视kinect点云转gridmap并做全局地图拼接的相关代码。运行程序请运行sh文件夹中相应的.sh文件.

    * 依赖:  

    ```sh
    官方grid_map包，安装参考<https://github.com/ANYbotics/grid_map>
    ```

    * TODO List
        - [x] 实时的每帧点云转gridmap(包含坐标转换)
        - [x] 全局地图拼接
        - [x] 使用滤波器生成表面法向量、坡度、粗糙度等信息层判断可通行区域，并根据可通行区域生成occupymap
        - [x] 同时输出Localmap、Globalmap和occupancy grid map
        - [x] 实现全过程参数化配置   


* [moon_landing包](./UAV/moon_landing)  
此功能包为无人机月面着陆仿真代码。MoonLanding.hpp为障碍识别、月面地标追踪的代码。moon_landing_node.cpp为降落过程的任务规划主代码。运行moon_landing.h即可运行仿真。

    * 依赖:  

    ```sh
    PX4/avoidance避障包。安装参考<https://github.com/PX4/avoidance>
    ```

    * TODO List
        - [x] 障碍检测
        - [x] 安全降落区域识别
        - [x] 分阶段的特征点（月面地标）追踪导航
        - [x] 最终降落时悬停进行月面扫描选取安全着陆点
        - [　] 分阶段后各阶段到全局地图的映射   
        - [　] 换算到真实距离     
        - [ ] 加入IMU进行数据融合 



* [grad_traj_optimization包]()  
此功能包为香港科技大学git上的轨迹生成和优化包。[github地址](http://github.com/HKUST-Aerial-Robotics/grad_traj_optimization)

## packages for UGV
* [test_octomap包](./UGV/test_octomap)  
    测试点云数据转octomap，并进行拼接的算法
    * 更新`test_octomap`添加无人机导航/turtlebot导航配置  
        * 查看`test_octomap`下的sh文件夹  
        1. pointcloud_to_octomap.sh用于无人机地面重建
        2. turtlebot_navi.sh用于turtlebot导航
        3. uav_navi.sh用于无人机导航
    
    *　更新turtlebot模型，添加６４线雷达

* [navi_algorithm包](./UGV/navi_algorithm)  
    实现各类导航算法包  

    * TO DO List:  
        - [ ] 二维导航算法  
        - [x] rrt  
        - [ ] rrt_connet(testing)  

* [scout_ros包](./UGV/scout_ros)  
  使用四驱小车进行导航gazebo仿真环境  
  安装参考https://github.com/westonrobot/scout_ros/tree/melodic  


    * 依赖:  

    ```sh
    sudo apt-get install ros-kinetic-gazebo-ros-control
    sudo apt install ros-kinetic-teleop-twist-keyboard
    sudo apt-get install ros-kinetic-joint-state-publisher-gui
    sudo apt install ros-kinetic-ros-controllers
    sudo apt install ros-kinetic-webots-ros
    sudo apt install ros-kinetic-navigation
    ```

    * 使用请进入`/scout_ros/navi_ros/sh`
        * scout_navi.sh　使用四驱小车导航

    * TODO List
        - [x] 四驱车模型  
        - [x] 相机模型添加
        - [x] 多线激光雷达添加

* [octomap_scout包](./UGV/octomap_scout)  
  使用scout四驱小车在gazebo仿真环境进行导航的包，独立于scout_ros。好处在于不用修改官方scout_ros包的内容就可以使用自定义的scout小车。
    
    * 依赖: 
        - scout_ros  
         1. 安装本文件夹中的包  `scout_ros` 
         2. 参考官方scout_ros包的melodic分支，安装参考<https://github.com/westonrobot/scout_ros/tree/melodic>  

        - velodyne
        ```sh
        sudo apt-get install ros-kinetic-velodyne*   
        ```
        - octomap

        ```sh
        sudo apt-get install ros-kinetic-octomap-ros #安装octomap
        sudo apt-get install ros-kinetic-octomap-msgs
        sudo apt-get install ros-kinetic-octomap-server
        sudo apt-get install ros-indigo-octomap-rviz-plugins
        ```  
        - navigation

        ```sh
        sudo apt install ros-kinetic-navigation
        ```

        - loam (三选一)  
        LOAM_velodyne　-- 安装参考<https://github.com/laboshinl/loam_velodyne>
        - A-LOAM (三选一)  
        ALoam -- 安装参考<https://github.com/HKUST-Aerial-Robotics/A-LOAM>
        - LeGO-LOAM  (三选一)  
        LeGO-LOAM -- 安装参考<https://github.com/RobustFieldAutonomyLab/LeGO-LOAM>


    
    * TODO List
        - [x] gazebo加载四驱车模型  
        - [x] kinect相机模型添加
        - [x] 多线激光雷达添加
        - [x] IMU传感器添加
        - [x] LOAM_velodyne初步实现
        - [x] aloam配置
        - [x] 基于octomap抽取二维地图，并构建costmap
        - [x] 使用navigation包导航
        - [x] 添加全局地图
        - [x] 在全局地图模式下，添加障碍层，使用了是双目的点云数据
        - [x] 在aloam上添加回环（位置回环与检测上下文回环）
        - [x] 在有全局点云地图和栅格地图的情况下，完成初始定位
        - [x] 使用全局点云裁剪的局部地图更新costmap


    * 使用请进入sh目录  

    ```
       ---- sh  
          -- scout_aloam_navi.sh 使用aloam进行建图与导航  
          -- scout_lego_navi.sh 使用lego进行建图与导航
          -- scout_aloam_navi_map.sh 在有全局地图的情况下，使用aloam进行建图与导航（完成了初始定位）
          -- scout_navi_octomap.sh使用loam进行导航（未充分测试，慎用）    
    ```

* [gridmap_scout包](./UGV/gridmap_scout)  
此功能包为无人小车上传感器数据转gridmap的功能包，且能够输出占据地图供Navigation导航包使用。pointcloud_to_gridmap.hpp为velodyne点云转gridmap,以及SLAM生成的全局地图同时生成全局grimap的相关代码。运行程序请运行sh文件夹中相应的.sh文件.

    * 依赖:  

    ```sh
    官方grid_map包，安装参考<https://github.com/ANYbotics/grid_map>
    ```

    * TODO List
        - [x] 实时的点云转gridmap(包含坐标转换)
        - [x] 全局地图生成
        - [x] 使用滤波器生成坡度、粗糙度等信息层判断可通行区域，并根据可通行区域生成occupymap供导航使用
        - [x] 实现全过程参数化配置 

* [octomap_navi](./UGV/octomap_navi)
此功能包为实现基于八叉树导航的小车功能包，区别于`octomap_scout包`基于八叉树抽取二维地图的方法

    * TODO List
        - [x] 实现基于八叉树的rrt前端路径查找
        - [ ] 实现八叉树上的通行走廊生成
        - [ ] 基于贝塞尔曲线的mini-jerk轨迹优化




