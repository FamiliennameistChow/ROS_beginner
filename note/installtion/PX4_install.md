# 如何正确安装ros_px4_gazebo仿真环境

-----
本教程将安装ros, px4工具链, 以及gazebo仿真环境

首先应该了解ubuntu, ros发行版, gazebo版本之间的关系
[参考这里](https://blog.csdn.net/ZhangRelay/article/details/79982187)

所以，给出对应版本的推荐（LTS):

> Ubuntu 14.04 Trusty Tahr     -  ROS Indigo Igloo        -  Gazebo 2.X  2014-2019

> Ubuntu 16.04 Xenial Xerus   -  ROS Kinetic Kame       -  Gazebo 7.X  2016-2021

> Ubuntu 18.04 Bionic Beaver -  ROS Melodic Morenia -  Gazebo 9.X  2018-2023

本教程将在`Ubuntu 16.04`上安装`ROS Kinetic`以及`px4 toolchain`  
**由于`ROS Kinetic`上的gazebo7与`px4 toolchain`中的gazebo9不兼容,所以安装要特别注意**

-----

## 安装px4_toolchain

[PX4 STIL](https://github.com/ukyan/AirSim/blob/master/docs/sitl.md)  

* 安装px4 toolchain

[tool chain](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux_ubuntu.html)
![toolchain](_v_images/20191022093216064_342500731.png)

请使用`ubuntu_sim_common_deps.sh`脚本, 该脚本不会安装`gazebo`,不然会将ros-kinetic中安装的gazebo7覆盖


```sh
#!/bin/bash

## Bash script for setting up a PX4 development environment on Ubuntu LTS (16.04).
## It can be used for installing simulators (only) or for installing the preconditions for Snapdragon Flight or Raspberry Pi.
##
## Installs:
## - Common dependencies and tools for all targets (including: Ninja build system, Qt Creator, pyulog)
## - FastRTPS and FastCDR
## - jMAVSim simulator dependencies
## - PX4/Firmware source (to ~/src/Firmware/)

# Preventing sudo timeout https://serverfault.com/a/833888
trap "exit" INT TERM; trap "kill 0" EXIT; sudo -v || exit $?; sleep 1; while true; do sleep 60; sudo -nv; done 2>/dev/null &

# Ubuntu Config
echo "We must first remove modemmanager"
sudo apt-get remove modemmanager -y


# Common dependencies
echo "Installing common dependencies"
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake build-essential genromfs ninja-build exiftool astyle -y
# make sure xxd is installed, dedicated xxd package since Ubuntu 18.04 but was squashed into vim-common before
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y
# Required python packages
sudo apt-get install python-argparse python-empy python-toml python-numpy python-dev python-pip -y
sudo -H pip install --upgrade pip
sudo -H pip install pandas jinja2 pyserial pyyaml
# optional python tools
sudo -H pip install pyulog

# Install FastRTPS 1.7.1 and FastCDR-1.0.8
fastrtps_dir=$HOME/eProsima_FastRTPS-1.7.1-Linux
echo "Installing FastRTPS to: $fastrtps_dir"
if [ -d "$fastrtps_dir" ]
then
    echo " FastRTPS already installed."
else
    pushd .
    cd ~
    wget https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-7-1/eprosima_fastrtps-1-7-1-linux-tar-gz -O eprosima_fastrtps-1-7-1-linux.tar.gz
    tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz eProsima_FastRTPS-1.7.1-Linux/
    tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz requiredcomponents
    tar -xzf requiredcomponents/eProsima_FastCDR-1.0.8-Linux.tar.gz
    cpucores=$(( $(lscpu | grep Core.*per.*socket | awk -F: '{print $2}') * $(lscpu | grep Socket\(s\) | awk -F: '{print $2}') ))
    (cd eProsima_FastCDR-1.0.8-Linux && ./configure --libdir=/usr/lib && make -j$cpucores && sudo make install)
    (cd eProsima_FastRTPS-1.7.1-Linux && ./configure --libdir=/usr/lib && make -j$cpucores && sudo make install)
    rm -rf requiredcomponents eprosima_fastrtps-1-7-1-linux.tar.gz
    popd
fi

# jMAVSim simulator dependencies
echo "Installing jMAVSim simulator dependencies"
sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y

# Clone PX4/Firmware
clone_dir=~/src
echo "Cloning PX4 to: $clone_dir."
if [ -d "$clone_dir" ]
then
    echo " Firmware already cloned."
else
    mkdir -p $clone_dir
    cd $clone_dir
    git clone https://github.com/PX4/Firmware.git
fi
```


* 编译px4 /Firmware, **这里安装1.8.2版本**

```
mkdir -p PX4
cd PX4
git clone https://github.com/PX4/Firmware.git
cd Firmware

git checkout v1.8.2

git submodule update --init --recursive

make posix_sitl_default

```
**如果要安装1.9.2**

```
git checkout v1.9.2

git submodule update --init --recursive

make px4_sitl_default
```

## 安装ros

[参考](https://www.cnblogs.com/liu-fa/p/5779206.html)

## mavros and mavlink 安装

[mavros安装](https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html)


## 在ros中使用python3[未测试]

```
pip3 install catkin-tools
pip3 install rospkg
sudo apt-get install python3-rosinstall
```