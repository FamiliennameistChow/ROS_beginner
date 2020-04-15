# ROS_beginner
包含了适用于UAV和UGV的多个ROS包，可以实现不同的功能

-----

# Table of Contents 

## packages for UAV 
* [drone_lora包](./drone_lora)  
此功能包为串口和socket使用包。src文件夹中为使用serial和socket的示例代码。

* [px4_mavros包](./px4_mavros)  
此功能包为基础学习和无人机控制包。包括基本的offboard模式控制无人机以及扩展而来的键盘控制无人机（共三个版本）。include文件夹内包含一个库文件。将此库文件作为头文件，便可以使用类中的成员函数控制无人机或获取无人机参数。src文件夹内drone_flight_modes_node.cpp为库文件使用方法示例代码。

* [vision包](./vision)  
此功能包为无人机视觉相关包。 
    ```
    img_pub.cpp为打开USB相机并发布ROS图像消息；  
    img_sub为接收ROS图像消息并显示；    
    mark_tracking为追踪降落相关代码；  
    detect_mark为地标识别相关代码； 
    red_detect为红色标靶识别代码，与planet_landing一起组成自主着陆比赛的功能代码。
    ```   
    运行程序请运行sh文件夹中相应的.sh文件.

* [grid_map_uav包](./grid_map_uav)  
此功能包为grid_map在无人机上的应用。depth_to_gridmap.hpp为深度图转grid_map并做全局地图拼接的相关代码。pointcloud_to_gridmap.hpp为kinect点云转grid_map并做全局地图拼接的相关代码。运行程序请运行sh文件夹中相应的.sh文件.

    * 依赖:  

    ```sh
    官方grid_map包，安装参考<https://github.com/ANYbotics/grid_map>
    ```

    * TODO List
        - [x] 实时的点云转gridmap  
        - [x] 全局地图拼接
        - [x] 使用滤波器生成表面法向量、坡度、粗糙度等信息层判断可通行区域，并根据可通行区域生成occupymap
        - [x] 同时输出Localmap和Globalmap
        - [x] 实现全过程参数化配置 
        - [ ] 生成costmap
        - [ ] 运动规划与导航

* [grad_traj_optimization包]()  
此功能包为香港科技大学git上的轨迹生成和优化包。[github地址](http://github.com/HKUST-Aerial-Robotics/grad_traj_optimization)

## packages for UGV
* [test_octomap包](./test_octomap)  
    测试点云数据转octomap，并进行拼接的算法
    * 更新`test_octomap`添加无人机导航/turtlebot导航配置  
        * 查看`test_octomap`下的sh文件夹  
        1. pointcloud_to_octomap.sh用于无人机地面重建
        2. turtlebot_navi.sh用于turtlebot导航
        3. uav_navi.sh用于无人机导航

* [navi_algorithm包](./navi_algorithm)  
    实现各类导航算法包  

    * TO DO List:  
        - [ ] 二维导航算法  
        - [x] rrt  
        - [ ] rrt_connet(testing)  

* [scout_ros包](./scout_ros)  
  使用四驱小车进行导航gazebo仿真环境  
  安装参考https://github.com/westonrobot/scout_ros/tree/melodic  


    * 依赖:  

    ```sh
    sudo apt-get install ros-kinetic-gazebo-ros-control
    sudo apt install ros-kinetic-teleop-twist-keyboard
    sudo apt-get install ros-kinetic-joint-state-publisher-gui
    sudo apt install ros-kinetic-ros-controllers
    ```

    * 使用请进入`/scout_ros/navi_ros/sh`
        * scout_navi.sh　使用四驱小车导航

    * TODO List
        - [x] 四驱车模型  
        - [x] 相机模型添加
        - [ ] 多线激光雷达添加

* [octomap_scout包](./octomap_scout)  
  使用scout四驱小车在gazebo仿真环境进行导航的包，独立于scout_ros。好处在于不用修改官方scout_ros包的内容就可以使用自定义的scout小车。
    
    * 依赖:  

    ```sh
    官方scout_ros包的melodic分支，安装参考<https://github.com/westonrobot/scout_ros/tree/melodic>  
    sudo apt-get install ros-kinetic-velodyne*   
    LOAM_velodyne－安装参考<https://github.com/laboshinl/loam_velodyne>
    ```
    
    * TODO List
        - [x] gazebo加载四驱车模型  
        - [x] kinect相机模型添加
        - [x] 多线激光雷达添加
        - [x] IMU传感器添加
        - [x] LOAM_velodyne初步实现
        - [ ] 生成costmap






