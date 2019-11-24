# turtlebot_gazebo_simulation

-----

使用gazebo 对turtlebot2进行仿真

> 环境:
> ubuntu 16.04 
> ros-kinetic
> gazebo7

------

[ubuntu16.04+kinetic 环境下进行turtlebot2仿真+gazebo正确显示](https://blog.csdn.net/sinat_37273779/article/details/82864564)

------

## 安装turtlebot2


```sh
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs ros-kinetic-turtlebot-simulator
```

* 安装`turtlebot_simulator`

```sh
cd ~/catkin_ws/src
git clone https://github.com/robotics-in-concert/rocon_qt_gui.git
git clone https://github.com/turtlebot/turtlebot_simulator.git

```

回到catkin_ws目录下执行catkin_make，但是发现显示缺少东西，这时候还需要下载两个依赖源;

```sh
sudo apt-get install pyqt4-dev-tools
sudo apt-get install pyqt5-dev-tools
```
