# ROS_beginner
包含了多个ROS包，可以完成不同的常用功能

-----

# Table of Contents
* [drone_flight_modes包](./drone_flight_modes)
    此功能包为无人机控制包。include文件夹内包含一个库文件。将此库文件作为头文件，便可以使用类中的成员函数控制无人机或获取无人机参数。src文件夹内为库文件使用方法示例代码。
    
* [drone_lora包](./drone_lora)
    此功能包为串口和socket使用包。src文件夹中为使用serial和socket的示例代码。

* [grad_traj_optimization包]()
    此功能包为香港科技大学git上的轨迹生成和优化包。[github地址](http://github.com/HKUST-Aerial-Robotics/grad_traj_optimization)

* [px4_mavros包](./px4_mavros)
    此功能包为基础学习包。包括基本的offboard模式控制无人机以及扩展而来的键盘控制无人机（共两个版本）。

* [vision包](./vision)
    此功能包为无人机视觉相关包。img_pub.cpp为打开USB相机并发布ROS图像消息；img_sub为接收ROS图像消息并显示；auto_landing为自主降落相关代码；mark_tracking为追踪降落相关代码；detect_mark为地标识别相关代码；red_detect为红色标靶识别代码，与planet_landing一起组成着陆比赛的功能代码。

* [grid_map_uav包](./grid_map_uav)
　　此功能包为grid_map在无人机上的应用。depth_to_gridmap.hpp为深度图转grid_map并做全局地图拼接的相关代码。

* [test_octomap包](./test_octomap)  
    测试点云数据转octomap，并进行拼接的算法

* [navi_algorithm包](./navi_algorithm包)  
    实现各类导航算法包  

    * TO DO List:  
        - [ ] 二维导航算法  
        - [x] rrt  
        - [ ] rrt_connet(testing)  






