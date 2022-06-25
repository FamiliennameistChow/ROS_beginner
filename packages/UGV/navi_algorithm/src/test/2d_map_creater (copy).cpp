#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>

using namespace std;



int main(int argc, char * argv[]) {

    ros::init(argc, argv, "gridMap");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher pub = nh_private.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);

    int obstacle_num, obstacle_len, obstacle_width;
    nh_private.param<int>("obstacle_num", obstacle_num, 50);
    nh_private.param<int>("obstacle_len", obstacle_len, 30);
    nh_private.param<int>("obstacle_len", obstacle_width, 30);

    nav_msgs::OccupancyGrid map;

    // 向Rviz发送的数据中一定要包含frame_id
    map.header.frame_id="map";
    map.header.stamp = ros::Time::now(); 
    map.info.resolution = 0.05;          // float32
    // 栅格地图分辨率对应栅格地图中一小格的长和宽
    map.info.width      = 10/map.info.resolution;           // uint32
    map.info.height     = 10/map.info.resolution;           // uint32

    // 设置地图起始坐标　默认为0
    map.info.origin.position.x = -(map.info.width*map.info.resolution)/2;
    map.info.origin.position.y = -(map.info.height*map.info.resolution)/2;

    int p[map.info.width*map.info.height] = {-1};   // [0,100] 只初始化了第一个元素
    p[21] = 100;  //(1, 1) map坐标
    p[20] = 100;  //(1, 0)

    p[30] = 100;
    p[31] = -1;
    p[32] = -1;

    // rid_map  [y*map.info.width+x]生成随机障碍
    for(int n=0; n<obstacle_num;n++)
    {
        int y = obstacle_len + rand()%(map.info.height-obstacle_len*2+1);
        int x = obstacle_len + rand()%(map.info.width-obstacle_len*2+1);
        int xy = rand()%(2);
        if(xy == 0)
        {
            for(int i=0; i<obstacle_len; i++)
            {
                p[(y+i)*map.info.width+x] = 100;
                p[(y+i)*map.info.width+x+1] = 100;
            }
        }else
        {
            for(int i=0; i<obstacle_len; i++)
            {
                p[y*map.info.width+x+i] = 100;
                p[(y+1)*map.info.width+x+i] = 100;
            }
        }
    }


    // p[int(floor((x-map.info.origin.position.x)/map.info.resolution) + floor((y-map.info.origin.position.y)/map.info.resolution))] = 100;

    std::vector<signed char> a(p, p+map.info.width*map.info.height);
    map.data = a;

    while (ros::ok())
    {
        pub.publish(map);
        cout << "origin: " << map.info.origin.position.x << " " << map.info.origin.position.y << endl;
    }
    // ros::shutdown();
    return 0;
}
