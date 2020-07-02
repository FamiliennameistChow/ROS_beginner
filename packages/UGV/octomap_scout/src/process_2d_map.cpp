#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

using namespace std;
#define PI 3.1415926
typedef Eigen::Vector3d Vector3;

nav_msgs::OccupancyGrid down_map, up_map, whole_map, elevation_map, global_map; 
nav_msgs::OccupancyGrid map_out, map_merge;
bool getData2xt = false;

// param from ros
int step; // search step  th_occupy_num
int th_free_num; //the num of cells threshold in search eara 
string method; // angle_method,  stdev_method, stdev_method_sparse;
float th_angle; // angle method / angle th
float th_stdev; // stdev_method / stdev th
bool global_map_detect; // if have global map


void down_mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    down_map=*msg;
}

void up_mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    up_map=*msg;
}

void whole_mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    whole_map=*msg;
}

void elevation_mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    elevation_map=*msg;
}

void global_mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("global map received");
    global_map=*msg;
    ROS_INFO("init form global map !");
    map_merge.header.frame_id=global_map.header.frame_id;
    map_merge.header.stamp = ros::Time::now();
    map_merge.info.resolution = global_map.info.resolution;
    map_merge.info.origin.position.x = global_map.info.origin.position.x;
    map_merge.info.origin.position.y = global_map.info.origin.position.y;
    map_merge.info.height = global_map.info.height;
    map_merge.info.width = global_map.info.width;
    map_merge.data.clear();
    map_merge.data.resize(global_map.info.width * global_map.info.height, -1);
}

void mapindexToWorld(int i, float& wx_, float& wy_, nav_msgs::OccupancyGrid& map){
    /**
     *  【example】:
     *  data为一维排列，假设其宽度为 width = 5。其index为
     *  [0 1 2 3 4 5 6 7 8 9 10 11 12 13 14]
     *
     *   转换为二维形式为:
     *     4  3  2  1  0
     *   <---------------
     *  [[ 4  3  2  1  0] | 0
     *   [ 9  8  7  6  5] | 1
     *   [14 13 12 11 10] | 2
     *
     *   其中inex 14 的map坐标应该为 (4, 2)  即是: 14 % 5 = 4； 14 / 5 = 2
     * */
    float origin_x_ = map.info.origin.position.x; // 这里的origin.position是以起点珊格的右上角世界坐标表示
    float origin_y_ = map.info.origin.position.y;
    float resolution_ = map.info.resolution;
    int width = map.info.width;
    int mx, my;
    mx = i % width;
    my = i / width;
    wx_ = (mx+0.5)* resolution_ + origin_x_; //加0.5使得　用该index所在珊格的中心world坐标来表示该index坐标
    wy_ = (my+0.5) * resolution_ + origin_y_;

}

void worldToMapindex(float wx_, float wy_, int& i, nav_msgs::OccupancyGrid& map)
{
    float origin_x_ = map.info.origin.position.x; // 这里的origin.position是以起点珊格的右上角世界坐标表示
    float origin_y_ = map.info.origin.position.y;
    float resolution_ = map.info.resolution;
    int width = map.info.width;
    int mx, my;
    mx = floor((wx_ - origin_x_)/resolution_);
    my = floor((wy_ - origin_y_)/resolution_);
    i = my*width + mx;
}

void processMapGradientSparse(int i, nav_msgs::OccupancyGrid& map,  nav_msgs::OccupancyGrid& map_){
/* search surrounding cell
search pattern: 
                    *
                  *****
                 ***#***
                  *****
                    * 
*/
    int num = 0;
    int sum = 0;
    vector<int> value;
    vector<int> idx;
    idx.push_back(i+1);
    idx.push_back(i-1);
    idx.push_back(i-map.info.width);
    idx.push_back(i-map.info.width-1);
    idx.push_back(i-map.info.width+1);
    idx.push_back(i-map.info.width-2);
    idx.push_back(i-map.info.width+2);
    idx.push_back(i+map.info.width);
    idx.push_back(i+map.info.width-1);
    idx.push_back(i+map.info.width+1);
    idx.push_back(i+map.info.width-2);
    idx.push_back(i+map.info.width+2);
    idx.push_back(i-2);
    idx.push_back(i+2);
    idx.push_back(i-2*map.info.width);
    idx.push_back(i+2*map.info.width);
    idx.push_back(i-3);
    idx.push_back(i+3);

    for (auto iter=idx.begin();iter!=idx.end();iter++)
    {
        if(*iter < 0 || *iter > map.data.size()){
            continue;
        }
        if(map.data[*iter] != 100 && map.data[*iter] != -1){
            num += 1;
            value.push_back(map.data[*iter]);
            sum += map.data[*iter];
        }
    }

    if(num > th_free_num){
        float mean = sum / num;
        double accum  = 0.0;
        for(auto it=value.begin(); it!=value.end(); it++){
            accum += (*it - mean)*(*it - mean);
        }
        float stdev = sqrt(accum/(num-1));
        if(stdev > th_stdev){
            for (auto iter_=idx.begin();iter_!=idx.end();iter_++){
                if(*iter_ < 0 || *iter_ > map_.data.size()){
                    continue;
                }
                map_.data[*iter_] = 100;
            }
            // map_.data[i] = 100;
        }else{
            for (auto iter_=idx.begin();iter_!=idx.end();iter_++){
                if(*iter_ < 0 || *iter_ > map_.data.size()){
                    continue;
                }
                map_.data[*iter_] = 0;
            }
        }
    }else{
        map_.data[i] = 100;
        // for (auto iter_=idx.begin();iter_!=idx.end();iter_++){
        //     if(*iter_ < 0 || *iter_ > map_.data.size()){
        //         continue;
        //     }
        //     map_.data[*iter_] = 100;
        // }
    }

}


void processMapGradient(int i, nav_msgs::OccupancyGrid& map,  nav_msgs::OccupancyGrid& map_){
/* search surrounding cell
search pattern: 
                    *
                  *****
                 ***#***
                  *****
                    * 
*/
    int num = 0;
    int sum = 0;
    vector<int> value;
    vector<int> self_idx;
    self_idx.push_back(i+1);
    self_idx.push_back(i-1);
    self_idx.push_back(i-map.info.width);
    self_idx.push_back(i-map.info.width-1);
    self_idx.push_back(i-map.info.width+1);
    self_idx.push_back(i-map.info.width-2);
    self_idx.push_back(i-map.info.width+2);
    self_idx.push_back(i+map.info.width);
    self_idx.push_back(i+map.info.width-1);
    self_idx.push_back(i+map.info.width+1);
    self_idx.push_back(i+map.info.width-2);
    self_idx.push_back(i+map.info.width+2);
    self_idx.push_back(i-2);
    self_idx.push_back(i+2);
    self_idx.push_back(i-2*map.info.width);
    self_idx.push_back(i+2*map.info.width);
    self_idx.push_back(i-3);
    self_idx.push_back(i+3);

    vector<vector<int> > idx(4,vector<int>(13));

    vector<int> direction;
    direction.push_back(i + step*map.info.width);
    direction.push_back(i - step*map.info.width);
    direction.push_back(i-step);
    direction.push_back(i+step);


    // Forward left right
    // init the idx vector
    for(int a=0; a<idx.size(); a++){
        idx[a].push_back(direction[a]);
        idx[a].push_back(direction[a] +1);
        idx[a].push_back(direction[a] -1);
        idx[a].push_back(direction[a]-map.info.width);
        idx[a].push_back(direction[a]+map.info.width);
        idx[a].push_back(direction[a]-map.info.width-1);
        idx[a].push_back(direction[a]-map.info.width+1);

        idx[a].push_back(direction[a]+map.info.width-1);
        idx[a].push_back(direction[a]+map.info.width+1);

        idx[a].push_back(direction[a]+2*map.info.width);
        idx[a].push_back(direction[a]-2*map.info.width);
        idx[a].push_back(direction[a]-2);
        idx[a].push_back(direction[a]+2);
    }

    vector<int> vec_tmp;
    int num_temp;
    int num_direction;
    num_direction = 0;
    for (auto iter=idx.begin();iter!=idx.end();iter++){
        vec_tmp = *iter;
        num_temp = 0;
        for(auto it=vec_tmp.begin(); it != vec_tmp.end(); it++){
            if(*it < 0 || *it > map.data.size()){
                continue;
            }
            if(map.data[*it] != 100 && map.data[*it] != -1){
                num_temp += 1;
            }
        }

        if(num_temp < th_free_num){
            // map_.data[i] = 100;
            num_direction += 1;
            for(auto it = vec_tmp.begin(); it != vec_tmp.end(); it++){
                if(*it < 0 || *it > map.data.size()){
                    continue;
                }
                map_.data[*it] = 100;
            }

            // for(int i = 0; i < 9; i++){
            //     int index = vec_tmp[i];
            //     if(vec_tmp[i] < 0 || vec_tmp[i] > map.data.size()){
            //         continue;
            //     }
            //     map_.data[index] = 100;
            // }
            continue;
        }
    }

    if(num_direction > 2){
        return;
    } 
    
    // stdev method <<<<
    for (auto iter=self_idx.begin();iter!=self_idx.end();iter++)
    {
        if(*iter < 0 || *iter > map.data.size()){
            continue;
        }
        if(map.data[*iter] != 100 && map.data[*iter] != -1){
            num += 1;
            value.push_back(map.data[*iter]);
            sum += map.data[*iter];
        }
    }

    if(num > th_free_num){
        float mean = sum / num;
        float accum  = 0.0;
        for(auto it=value.begin(); it!=value.end(); it++){
            accum += (*it - mean)*(*it - mean);
        }
        float stdev = sqrt(accum/(num-1));
        if(stdev > th_stdev){
            for (auto iter_=self_idx.begin();iter_!=self_idx.end();iter_++){
                if(*iter_ < 0 || *iter_ > map_.data.size()){
                    continue;
                }
                map_.data[*iter_] = 100;
            }
            // map_.data[i] = 100;
        }else{
            for (auto iter_=self_idx.begin();iter_!=self_idx.end();iter_++){
                if(*iter_ < 0 || *iter_ > map_.data.size()){
                    continue;
                }
                map_.data[*iter_] = 0;
            }
        }
    }else{
        map_.data[i] = 100;
    }

}

void processMapHole(int i, nav_msgs::OccupancyGrid& map, nav_msgs::OccupancyGrid& map_){
/* search surrounding cell
search pattern: 
                    *
                   *** 
                  **#** 
                   ***
                    * 
*/
    int num = 0;
    vector<int> idx;
    idx.push_back(i);
    idx.push_back(i+1);
    idx.push_back(i-1);
    idx.push_back(i-map.info.width);
    idx.push_back(i-map.info.width-1);
    idx.push_back(i-map.info.width+1);
    idx.push_back(i+map.info.width);
    idx.push_back(i+map.info.width-1);
    idx.push_back(i+map.info.width+1);
    idx.push_back(i-2);
    idx.push_back(i+2);
    idx.push_back(i-2*map.info.width);
    idx.push_back(i+2*map.info.width);

    for (auto iter=idx.begin();iter!=idx.end();iter++)
    {
        if(*iter < 0 || *iter > map.data.size()){
            continue;
        }
        if(map.data[*iter] != 100 && map.data[*iter] != -1){
            num += 1;
        }
    }
    if(num < th_free_num){
        for (auto iter_=idx.begin();iter_!=idx.end();iter_++){
            if(*iter_ < 0 || *iter_ > map_.data.size()){
                continue;
            }
            map_.data[*iter_] = 100;
        }
    }

}


void processMapGradientV(int i, nav_msgs::OccupancyGrid& map,  nav_msgs::OccupancyGrid& map_){
/* search surrounding cell
// search pattern: 
//        #                *
//                        *** 
//     #  #  #           **#** 
//                        ***
//        #                * 
*/

    vector<int> fill_idx;
    vector<vector<int> > idx(4,vector<int>(9));
    int center_value = map.data[i];   //hegiht of the cell
    float res = map.info.resolution;
    vector<int> direction;

    direction.push_back(i + step*map.info.width);
    direction.push_back(i - step*map.info.width);
    direction.push_back(i - step);
    direction.push_back(i + step);

    fill_idx.push_back(i);
    fill_idx.push_back(i+1);
    fill_idx.push_back(i-1);
    fill_idx.push_back(i-map.info.width);
    fill_idx.push_back(i-map.info.width-1);
    fill_idx.push_back(i-map.info.width+1);
    fill_idx.push_back(i+map.info.width);
    fill_idx.push_back(i+map.info.width-1);
    fill_idx.push_back(i+map.info.width+1);
    fill_idx.push_back(i+2*map.info.width);
    fill_idx.push_back(i-2*map.info.width);
    fill_idx.push_back(i-2);
    fill_idx.push_back(i+2);

    // ROS_INFO("direction size is %d", int(direction.size()));

    // Forward back left right
    // init the idx vector
    for(int a=0; a<idx.size(); a++){
        idx[a].push_back(direction[a]);
        idx[a].push_back(direction[a] +1);
        idx[a].push_back(direction[a] -1);
        idx[a].push_back(direction[a]-map.info.width);
        idx[a].push_back(direction[a]-map.info.width-1);
        idx[a].push_back(direction[a]-map.info.width+1);
        idx[a].push_back(direction[a]+map.info.width);
        idx[a].push_back(direction[a]+map.info.width-1);
        idx[a].push_back(direction[a]+map.info.width+1);

        // idx[a].push_back(direction[a]+2*map.info.width);
        // idx[a].push_back(direction[a]-2*map.info.width);
        // idx[a].push_back(direction[a]-2);
        // idx[a].push_back(direction[a]+2);
    }

    // ROS_INFO("[GradientV]-------------------------------------------");

    vector<int> vec_tmp;
    vector<float> mean_value;
    int num;
    float sum;
    float angle;
    int num_direction = 0; 
    int count = 0;
    int invaild_count = 10;

    for (auto iter=idx.begin();iter!=idx.end();iter++){
        vec_tmp = *iter;
        count++;
        num = 0;
        sum = 0.0;
        for(auto it=vec_tmp.begin(); it != vec_tmp.end(); it++){
            if(*it < 0 || *it > map.data.size()){
                continue;
            }
            if(map.data[*it] != 100 && map.data[*it] != -1){
                num += 1;
                sum += map.data[*it];
            }
        }

        if(num < th_free_num){
            num_direction += 1;
            for(auto it = vec_tmp.begin(); it != vec_tmp.end(); it++){
                if(*it < 0 || *it > map.data.size()){
                    continue;
                }
                map_.data[*it] = -1;
            }
            //more than two direction have no value , the Gradient will will not be calculated in the cell
            if(num_direction >= 2){ 
                if (global_map_detect)
                {
                    map_.data[i] = -1;
                }else
                {
                    map_.data[i] = 100;
                }
                // ROS_INFO("cell normal invaild----");
                return;
            }
            if(num_direction == 1){
                mean_value.push_back(center_value);
                invaild_count = count;
                // ROS_INFO("invaild_count is %d", invaild_count);
            }
        }else
        {
            // ROS_INFO("mean_value : %f", sum/num);
            mean_value.push_back(sum/num);
        } // one direction end;

    }  // direction end

    // method to caculate normal 
    //         -----> x (width)
    //         |      #         1
    //  height |   #  #  #    3   4
    //         |      #         2
    //         y    

    double distanceX = 2* step * res;
    double distanceY = 2* step * res;
    Eigen::Vector3d normalVectorPositiveAxis_ = Vector3::UnitZ();

    if(invaild_count == 1 || invaild_count == 2){
        distanceY = step * res;
    }
    if(invaild_count == 3 || invaild_count == 4){
        distanceX = step * res;
    }

    // ROS_INFO("mean_value : %f, %f, %f, %f", mean_value[0], mean_value[1], mean_value[2], mean_value[3]);
    // ROS_INFO("center_value : %d", center_value);

    Vector3 normalVector = Vector3::Zero();
    // X DIRECTION
    normalVector(0) = (mean_value[0] - mean_value[1]) / (distanceX * 10);
    // Y DIRECTION
    normalVector(1) = (mean_value[2] - mean_value[3]) / (distanceY * 10);
    // Z DIRECTION
    normalVector(2) = +1;
    // ROS_INFO("normalVector : %f, %f, %f", normalVector(0), normalVector(1), normalVector(2));

    normalVector.normalize();
    // ROS_INFO("normalVector normalize : %f, %f, %f", normalVector(0), normalVector(1), normalVector(2));

    if (normalVector.dot(normalVectorPositiveAxis_) < 0.0) {
      normalVector = -normalVector;
    }

    angle = acos(normalVector.z())*180.0f/PI;
    // ROS_INFO("normalVector normalize z: %f", normalVector.z());
    // ROS_INFO("ANGLE is %f", angle);


    if(angle > th_angle){
        // map_.data[i] = 100;
        for(auto fill_it = fill_idx.begin(); fill_it != fill_idx.end(); fill_it++){
            if(*fill_it < 0 || *fill_it > map.data.size()){
                continue;
            }
            map_.data[*fill_it] = 100;
        }
 
        return;
    }else{
        for(auto fill_it = fill_idx.begin(); fill_it != fill_idx.end(); fill_it++){
            if(*fill_it < 0 || *fill_it > map.data.size()){
                continue;
            }
            map_.data[*fill_it] = 0;
        }
    }

    
}

int main(int argc, char * argv[]) {

    ros::init(argc, argv, "gridmap_process");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //load param
    nh_private.param<int>("step", step, 5);
    nh_private.param<int>("th_free_num", th_free_num, 3);
    nh_private.param<string>("method", method, string("stdev_method_sparse"));
    nh_private.param<float>("th_angle", th_angle, 30.0);
    nh_private.param<float>("th_stdev", th_stdev, 2.0);
    nh_private.param<bool>("global_map_detect", global_map_detect, true);
    

    ros::Subscriber down_map_sub = nh.subscribe("/projected_down_map", 100 ,down_mapCallBack);	
    ros::Subscriber up_map_sub = nh.subscribe("/projected_up_map", 100 ,up_mapCallBack);	
    ros::Subscriber whole_map_sub = nh.subscribe("/projected_whole_map", 100 ,whole_mapCallBack);
    ros::Subscriber elevation_map_sub = nh.subscribe("/projected_map", 100 , elevation_mapCallBack);	
    ros::Subscriber global_map_sub = nh.subscribe("/map", 100 , global_mapCallBack);

    ros::Publisher map_pub = nh_private.advertise<nav_msgs::OccupancyGrid>("/detect_map", 10);
    ros::Publisher map_octo_pub = nh_private.advertise<nav_msgs::OccupancyGrid>("/octo_map", 10);
      
    // if (global_map_detect)
    // {
    //     ROS_INFO("init form global map !");
    //     map_merge.header.frame_id=global_map.header.frame_id;
    //     map_merge.header.stamp = ros::Time::now();
    //     map_merge.info.resolution = global_map.info.resolution;
    //     map_merge.info.origin.position.x = global_map.info.origin.position.x;
    //     map_merge.info.origin.position.y = global_map.info.origin.position.y;
    //     map_merge.info.height = global_map.info.height;
    //     map_merge.info.width = global_map.info.width;
    //     map_merge.data.clear();
    //     map_merge.data.resize(global_map.info.width * global_map.info.height, -1);
    // }

    // wait until map is received, when a map is received, mapData.header.seq will not be < 1  
    while (elevation_map.header.seq<1 or elevation_map.data.size()<1){
        ROS_INFO("waiting for map !");
        ros::spinOnce();  
        ros::Duration(0.1).sleep();
        }
    
    map_out.header.frame_id=elevation_map.header.frame_id;
    map_out.header.stamp = ros::Time::now();
    map_out.info.resolution = elevation_map.info.resolution;
    map_out.info.origin.position.x = elevation_map.info.origin.position.x;
    map_out.info.origin.position.y = elevation_map.info.origin.position.y;
    

    ofstream file_out("/home/bornchow/AMOV_WORKSPACE/moon_ws/src/octomap_scout/launch/test/data.txt");
    int c = 0;

    while (ros::ok())
    {
       
        ROS_INFO("process....");
        // if (elevation_map.data.size() != global_map.data.size()){
        //     ROS_INFO("DOWN map IS NOT match whole map!!!");
        //     ROS_INFO("wohle map info ->  size: %zu, width: %d x height: %d", 
        //     whole_map.data.size(), whole_map.info.width, whole_map.info.height);
        //     ROS_INFO("down map info ->  size: %zu, width: %d x height: %d", 
        //     down_map.data.size(), down_map.info.width, down_map.info.height);
        //     ROS_INFO("up map info ->  size: %zu, width: %d x height: %d", 
        //     up_map.data.size(), up_map.info.width, up_map.info.height);
        // }

        if (global_map_detect)
        {
            if (elevation_map.data.size() != global_map.data.size())
            {
                ROS_INFO("elevation map IS NOT match global map!!!");
                ROS_INFO("elevation map info ->  size: %zu, width: %d x height: %d, origin_x: %f , origin_y: %f", 
                elevation_map.data.size(), elevation_map.info.width, elevation_map.info.height, 
                elevation_map.info.origin.position.x, elevation_map.info.origin.position.y);
                ROS_INFO("global map info ->  size: %zu, width: %d x height: %d, origin_x: %f , origin_y: %f", 
                global_map.data.size(), global_map.info.width, global_map.info.height, 
                global_map.info.origin.position.x, global_map.info.origin.position.y);
            }
            
        }
        

        // init to unknown:
        map_out.info.width = elevation_map.info.width;
        map_out.info.height = elevation_map.info.height;
        map_out.info.resolution = elevation_map.info.resolution;
        map_out.info.origin.position.x = elevation_map.info.origin.position.x;
        map_out.info.origin.position.y = elevation_map.info.origin.position.y;
        map_out.data.clear();
        map_out.data.resize(elevation_map.info.width * elevation_map.info.height, -1);

        // // prcess hole
        // for(int i = elevation_map.info.width; i< elevation_map.data.size() - elevation_map.info.width; i++)
        // {
        //     // ROS_INFO("value is: %d", down_map.data[i]);
        //     if(elevation_map.data[i] == 100){
        //         // ROS_INFO("id: %d is hole", i);
        //         int resetValue = 0;
        //         processMapValue(i, elevation_map, resetValue);
                
        //         map_out.data[i] = resetValue;
        //         // ROS_INFO("changed value to %d", resetValue);
                
        //     }else{
        //         map_out.data[i] = elevation_map.data[i];
        //     }
        // }


        // prcess gradient
        for(int i = 3*elevation_map.info.width; i< elevation_map.data.size() - 3*elevation_map.info.width; i++)
        {
            // ROS_INFO("value is: %d", down_map.data[i]);
            if(elevation_map.data[i] != 100 && elevation_map.data[i] != -1){
                // ROS_INFO("id: %d is hole", i); angle_method,  stdev_method
                if (method == "angle_method"){
                    processMapGradientV(i, elevation_map, map_out);
                }else if(method == "stdev_method_sparse"){
                    processMapGradientSparse(i, elevation_map, map_out);
                }else if (method == "stdev_method"){
                    processMapGradient(i, elevation_map, map_out);
                }else
                {
                    ROS_INFO("the method is NOT matched. suported methods are angle_method/stdev_method");
                }
            }
            // if(elevation_map.data[i] == -1){
            //     map_out.data[i] == -1;
            // }
        }

        // // prcess hole
        // for(int i = 3*elevation_map.info.width; i< elevation_map.data.size() - 3*elevation_map.info.width; i++)
        // {
        //     // ROS_INFO("value is: %d", down_map.data[i]);
        //     if(elevation_map.data[i] == 100){
        //         // ROS_INFO("id: %d is hole", i);
        //         processMapHole(i, elevation_map, map_out);
        //         // ROS_INFO("changed value to %d", resetValue);
                
        //     }
        // }



        //process down map
        // for(int i=0; i< down_map.data.size(); i++)
        // {
        //     // ROS_INFO("value is: %d", down_map.data[i]);
        //     if(down_map.data[i] == 100){
        //         // ROS_INFO("id: %d is hole", i);
        //         map_out.data[i] = 100;
        //     }
        //     if(down_map.data[i] == -1 || down_map.data[i] == 0){
        //         // ROS_INFO("id: %d is NOT hole", i);
        //         map_out.data[i] = 1;
        //     }
        // }

        if (global_map_detect)
        {
            ROS_INFO("map merge!!!!!");
            ROS_INFO("map_merge info ->  size: %zu, width: %d x height: %d, origin_x: %f , origin_y: %f", 
            map_merge.data.size(), map_merge.info.width, map_merge.info.height, 
            map_merge.info.origin.position.x, map_merge.info.origin.position.y);
            // map merge
            float wx, wy;
            int map_merge_ind;
            map_merge.data = global_map.data;
            for(int i = 3*map_out.info.width; i< map_out.data.size() - 3*map_out.info.width; i++){
                if (map_out.data[i] == 100)
                {
                    // ROS_INFO("map out index: %d", i);
                    mapindexToWorld(i, wx, wy, map_out);
                    worldToMapindex(wx, wy, map_merge_ind, map_merge);
                    map_merge.data[map_merge_ind] = 100;
                    // ROS_INFO("map merge index: %d", map_merge_ind);
                }
                if (map_out.data[i] == 0)
                {
                    mapindexToWorld(i, wx, wy, map_out);
                    worldToMapindex(wx, wy, map_merge_ind, map_merge);
                    if (map_merge.data[map_merge_ind] == -1)
                    {
                        map_merge.data[map_merge_ind] = 0;
                    }
                    
                }
                
            
            }
            map_pub.publish(map_merge);
            map_octo_pub.publish(map_out);
        }else
        {
            map_pub.publish(map_out);
        }
        
        if(getData2xt){
            c += 1;
            // ROS_INFO("c is:   %d", c);
            if(c == 10000){
                for(int i=0; i< elevation_map.data.size(); i++){
                    if(i % elevation_map.info.width == 0){
                        file_out << endl;
                    }
                    file_out <<" "<< to_string(elevation_map.data[i]);
                    if(elevation_map.data[i] != 0){
                        ROS_INFO("data is : %d", elevation_map.data[i]);
                    }
                }
            file_out.close();
            ROS_INFO("data get done!!!");
            }
        }


        ros::spinOnce();

    }
    
    return 0;
}
