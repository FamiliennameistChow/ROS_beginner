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

* [sjtu_game](./UAV/sjtu_game)  
交大比赛仿真，使用`Ｆast_Planner`做路径规划, yolo做识别
    * 依赖:
        - Fast_Planner  
         1. 安装参考　https://github.com/HKUST-Aerial-Robotics/Fast-Planner  
        
        - YOLO darknet_ros


* [target_tracking](./UAV/target_tracking)
此功能包是为了后面做目标追踪的功能包,目前只加入了初步场景,添加了scout_uav_sim.launch文件用来启动场景.
    * 依赖/配置

        参考UGV里的Scout_ros配置,安装相应依赖,就可以启动scout_uav_sim.launch了,中间要去安装一个激光雷达的包.
        
        找到小车的配置文件在:./UGV/scout_ros/scout_description/urdf/scout_v2.xacro
        
        添加标靶信息:将./UAV/urdf/model.urdf文件内容复制到小车配置文件,就可以看到一个小车上顶着一个图片  

        ![pic](./UAV/target_tracking/models/box_target_green/materials/textures/start_pad3.png)

        无人机和无人车的模型都加载到一个world里去,并且利用键盘控制都可以操控两个vehicle

* [target_landing](./UAV/target_landing)
此功能包用于集群仿真，添加了无人车、多架无人机（3架/9架）、地面靶标，分别通过scout_land_sim_3.launch 和scout_land_sim_9.launch 来启动场景。

    * 依赖/配置

        * 配置Scout_ros，安装相应依赖，参考UAV 里的Target_tracking / 参考官方scout_ros包的melodic分支，安装参考<https://github.com/westonrobot/scout_ros/tree/melodic>

        * gazebo 模型
        
            可在本Git 的gazebo/models/target_landing 文件夹下找到，放入`~/.gazebo/models/`目录。

        * `安装xmlstarlet`
            
            px4/Firmware的版本在1.10.2及以上提供了以sdf文件为模型的多机启动文件（之前版本使用urdf文件添加），依赖xmlstarlet 工具。

                sudo apt update
                sudo apt install xmlstarlet
        
        * `添加相机`
        
            将本Git 的gazebo/models/target_landing 文件夹下的iris_cam 和kinect_self 模型文件夹复制到px4 模型目录下`～/px4/Firmware/Tools/sitl_gazebo/models`。

            * iris_cam 模型的sdf 文件用于加载无人机和所需要的相机，相机可以根据任务需要自己修改。
            
            * kinect_self 模型即为波波教程中添加的双目相机的模型，为多机修改了部分细节。

        * 安装darknet_ros包
            - 安装darknet_ros包

            - 配置
                在本项目下`third_party\darknet_ros_simulation_config`找到配置文件  

                1. 将`car123.yaml` 和 `ros.yaml` 放在　`darknet_ros\config`下  
                2. 将`darknet_ros.launch` 放在　`darknet_ros\launch`下  
                3. 将`yolov3_tiny_123car.cfg` 放在　`darknet_ros\yolo_network_config\cfg`下
                4. 将`yolov3_tiny_mulitUAV_40000.weights` 放在　`darknet_ros\yolo_network_config\weights`下
    
    * TODO List:
        - [x] 单侦察机代码逻辑 `attacker.cpp`
        - [x] 攻击机代码逻辑 `scout_plane1.cpp`
        - [ ] 多侦查机代码逻辑

    * 侦察机攻击机协同侦打
        1. 启动场景与攻击机  
            进入sh目录

            ```
                --sh
                    -- arm_7_attacker.sh 启动场景并所有攻击机进入待命状态
                    -- arm_7_attacker_withou_gazebo.sh　所有攻击机进入待命状态(需要先自动启动场景)
            ```

            ```
                ./arm_7_attacker.sh
            ```

            等待场景启动，并看到七架飞机take-off
        2. 启动侦察机

            ```
                roslaunch target_landing uav_scout_plane.launch
            ```
        
        3. 启动识别节点

            ```
                roslaunch darknet_ros darknet_ros.launc
            ```

* [riverdetect](./UAV/riverdetect)
此功能包用于河道巡检，运行riverdetect.sh文件即可启动。启动后在gazebo界面左侧World 竖框内找到Models/iris_rplidar，右键选择Move To 即可找到飞机。
   
   * gazebo环境配置

        打开gazebo/models/river_simulation_config 文件夹，按照config.md文件配置方针环境。

-----


------


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
        - [x] rrt_connet
        - [x] rrt_star
        - [x] A-star
        - [x] Dijkstra
        - [x] JPS

* [scout_ros包](./UGV/scout_ros)  
  使用四驱小车进行导航gazebo仿真环境  
  安装参考https://github.com/westonrobot/scout_ros/tree/melodic  


    * 依赖:  

    ```sh
    sudo apt-get install ros-kinetic-gazebo-ros-control
    sudo apt install ros-kinetic-teleop-twist-keyboard
    sudo apt-get install ros-kinetic-joint-state-publisher-gui
    sudo apt install ros-kinetic-ros-controllers
    sudo apt install ros-$ROS_DISTRO-webots-ros
    sudo apt install ros-$ROS_DISTRO-navigation
    sudo apt-get install ros-$ROS_DISTRO-velodyne*
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
        - [x] 提高位姿估计频率


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
        - [ ] 优化时间分配问题，提高求解的鲁棒性 


        



