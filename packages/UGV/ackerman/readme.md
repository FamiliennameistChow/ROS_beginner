# 阿克曼底盘的仿真包——服务于园区项目

## 使用方法：

将 models 中的  放入 ~/.gazebo/models中。

其中 kinect 这个 模型原本的包里应该有，但是没有传感器插件，所以把官方的替换掉

```
roslaunch ackerman_sim_gazebo ackerman_sim.launch       // 文件中可以更换场景，以及选择时候使用系统的 里程计

rosrun acman_navigation trajectory_server               // 轨迹跟踪的节点

rosrun acman_navigation trajectory_test                 // 发布一个测试轨迹
```



也可以使用键盘控制 车体运动

```
rosrun ackerman_sim_gazebo keyboard_controll
```


还有非常多的 TODO： 需要完善
