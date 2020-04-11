# SDK for AgileX Scout Mobile Base

Copyright (c) 2019 [WestonRobot](https://www.westonrobot.com/)

## Introduction

This software packages provides a C++ interface to communicate with the Scout mobile base, for sending commands to the robot and acquiring the latest robot state. The SDK works on both x86 and ARM platforms.

Generally, you only need to instantiate an object of "class ScoutBase", then use the object to programmatically control the robot. Internally, class ScoutBase manages two background threads, one to process CAN/UART messages of the robot state and accordingly update state variables in the ScoutState data structure, and the other to maintain a 50Hz loop and send the latest command to the robot base. User can iteratively perform tasks in the main thread and check the robot state or set control commands. 

Refer to "src/apps/scout_demo" for an example.

## Package Structure

* apps: demo to illustrate how to use the SDK, scout_monitor is a TUI application to monitor states of Scout
* comm/async_io: manages raw data communication with robot
* comm/scout_protocol: encoding and decoding of Scout UART/CAN protocols
* scout_base: interface to send command to robot and receive robot state
* third_party
    - asio: asynchronous IO management (serial and CAN)
    - googletest: for unit tests only (not required otherwise)

## Setup CAN-To-USB adapter 
 
The instructions work for stm32f0-based adapter with [candleLight](https://github.com/HubertD/candleLight_fw) firmware on a host computer running Linux. (Refer to limitations listed at the bottom for more details.)

1. Enable gs_usb kernel module
    ```
    $ sudo modprobe gs_usb
    ```
2. Bringup can device
   ```
   $ sudo ip link set can0 up type can bitrate 500000
   ```
3. If no error occured during the previous steps, you should be able to see the can device now by using command
   ```
   $ ifconfig -a
   ```
4. Install and use can-utils to test the hardware
    ```
    $ sudo apt install can-utils
    ```
5. Testing command
    ```
    # receiving data from can0
    $ candump can0
    # send data to can0
    $ cansend can0 001#1122334455667788
    ```

Two scripts inside the "./scripts" folder are provided for easy setup. You can run "./setup_can2usb.bash" for the first-time setup and run "./bringup_can2usb.bash" to bring up the device each time you unplug and re-plug the adapter.

## Build SDK

Install compile tools

```
$ sudo apt install build-essential cmake
```

If you want to build the TUI monitor tool, install libncurses

```
$ sudo apt install libncurses5-dev
```

Configure and build

```
$ cd scout_sdk 
$ mkdir build
$ cd build
$ cmake ..
$ make
```

## Known Limitations

1. The CAN interface requires the hardware to appear as a CAN device in the system. You can use the command "ifconfig" to check the interface status. For example, you may see something like

    ```
    can1: flags=193<UP,RUNNING,NOARP>  mtu 16
            unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 10  (UNSPEC)
            RX packets 4751634  bytes 38013072 (36.2 MiB)
            RX errors 0  dropped 0  overruns 0  frame 0
            TX packets 126269  bytes 1010152 (986.4 KiB)
            TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
            device interrupt 43
    ```
    
    If you use your own CAN-to-USB adapter, make sure it supports slcan or can be brought up as a native CAN device (for example, CANable https://www.canable.io/). Some adapters may use a custom-defined protocol and appear as a serial device in Linux. In such a case, you will have to translate the byte stream between CAN and UART by yourself. It would be difficult for us to provide support for them since not all manufacturers define this protocol in the same way.

<!-- 
2. Release v0.1 of this SDK provided a serial interface to talk with the robot. Front/rear light on the robot cannot be controlled and only a small subset of all robot states can be acquired through that interface. Full support of the serial interface is still under development and requires additional work on both the SDK and firmware sides.
-->

## Reference

* [CAN command reference in Linux](https://wiki.rdu.im/_pages/Notes/Embedded-System/Linux/can-bus-in-linux.html)