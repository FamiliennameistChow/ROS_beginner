#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include <fstream>

using namespace std;
#define PI 3.1415926
nav_msgs::OccupancyGrid down_map, up_map, whole_map, elevation_map; 
nav_msgs::OccupancyGrid map_out;
bool getData2xt = false;

// param from ros
int step; // search step  th_occupy_num
int th_free_num; //the num of cells threshold in search eara 
string method; // angle_method,  stdev_method, stdev_method_sparse;
float th_angle; // angle method / angle th
float th_stdev; // stdev_method / stdev th


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
        double stdev = sqrt(accum/(num-1));
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
    }

}


void processMapGradient(int i, nav_msgs::OccupancyGrid& map,  nav_msgs::OccupancyGrid& map_){
/* search surrounding cell
search pattern: 
                    *
                   *** 
                  **#** 
                   ***
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
    self_idx.push_back(i+map.info.width);
    self_idx.push_back(i+map.info.width-1);
    self_idx.push_back(i+map.info.width+1);
    self_idx.push_back(i-2);
    self_idx.push_back(i+2);
    self_idx.push_back(i-2*map.info.width);
    self_idx.push_back(i+2*map.info.width);

    vector<vector<int> > idx(4,vector<int>(9));

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
            map_.data[i] = 100;
            num_direction += 1;
            for(auto it = vec_tmp.begin(); it != vec_tmp.end(); it++){
                if(*it < 0 || *it > map.data.size()){
                    continue;
                }
                map_.data[*it] = 100;
            }
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

// void processMapValue(int i, nav_msgs::OccupancyGrid& map, int& result){
// /* search surrounding cell
// search pattern: 
//                     *
//                    *** 
//                   **#** 
//                    ***
//                     * 
// */
//     int num = 0;
//     int sum = 0;
//     vector<int> value;
//     vector<int> idx;
//     idx.push_back(i+1);
//     idx.push_back(i-1);
//     idx.push_back(i-map.info.width);
//     idx.push_back(i-map.info.width-1);
//     idx.push_back(i-map.info.width+1);
//     idx.push_back(i+map.info.width);
//     idx.push_back(i+map.info.width-1);
//     idx.push_back(i+map.info.width+1);
//     idx.push_back(i-2);
//     idx.push_back(i+2);
//     idx.push_back(i-2*map.info.width);
//     idx.push_back(i+2*map.info.width);

//     for (auto iter=idx.begin();iter!=idx.end();iter++)
//     {
//         if(*iter < 0 || *iter > map.data.size()){
//             continue;
//         }
//         if(map.data[*iter] != 100 && map.data[*iter] != -1){
//             num += 1;
//             value.push_back(map.data[*iter]);
//             sum += map.data[*iter];
//         }
//     }

//     if(num > 3){
//          result = int(sum / num);
//     }else{
//         result = 100;
//     }

// }


void processMapGradientV(int i, nav_msgs::OccupancyGrid& map,  nav_msgs::OccupancyGrid& map_){
/* search surrounding cell
// search pattern: 
//        #                *
//                        *** 
//     #  #  #           **#** 
//                        ***
//        #                * 
*/
    // vector<float> mean_value; // mean_value of 9 surrounding cells
    vector<int> fill_idx;
    vector<vector<int> > idx(4,vector<int>(9));
    int center_value = map.data[i]; //hegiht of the cell
    // vector<int> idxF, idxL, idxR;
    // int iF, iL, iR;
    // iF = i + 3*map.info.width;
    // iL = i - 3;
    // iR = i + 3;
    vector<int> direction;

    direction.push_back(i + step*map.info.width);
    direction.push_back(i - step*map.info.width);
    direction.push_back(i-step);
    direction.push_back(i+step);

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

    // Forward left right
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

    ROS_INFO("[GradientV] init sreach finished!!!");

    vector<int> vec_tmp;
    vector<int> stdev_value;
    int num;
    float sum;
    float mean_value;
    float angle;

    for (auto iter=idx.begin();iter!=idx.end();iter++){
        vec_tmp = *iter;
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
            map_.data[i] = 100;
            for(auto it = vec_tmp.begin(); it != vec_tmp.end(); it++){
                if(*it < 0 || *it > map.data.size()){
                    continue;
                }
                map_.data[*it] = 100;
            }
            return;
        }else
        {
            //angle method >>>>
            mean_value = sum / num;
            angle = atan(abs(mean_value - center_value)/step)*180.0f/PI; //
            // ROS_INFO("ANGLE is %f", angle);
            if(angle > th_angle){
                map_.data[i] = 100;
                // for(auto it = vec_tmp.begin(); it != vec_tmp.end(); it++){
                //     if(*it < 0 || *it > map.data.size()){
                //         continue;
                //     }
                //     map_.data[*it] = 100;
                // }
                return;
            }else{
                for(auto fill_it = fill_idx.begin(); fill_it != fill_idx.end(); fill_it++){
                    if(*fill_it < 0 || *fill_it > map.data.size()){
                        continue;
                    }
                    map_.data[*fill_it] = 0;
                }
            }
            // angle method <<<< 
        }
        // one direction end;
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
    

    ros::Subscriber down_map_sub = nh.subscribe("/projected_down_map", 100 ,down_mapCallBack);	
    ros::Subscriber up_map_sub = nh.subscribe("/projected_up_map", 100 ,up_mapCallBack);	
    ros::Subscriber whole_map_sub = nh.subscribe("/projected_whole_map", 100 ,whole_mapCallBack);	
    ros::Subscriber elevation_map_sub = nh.subscribe("/projected_map", 100 , elevation_mapCallBack);
    ros::Publisher map_pub = nh_private.advertise<nav_msgs::OccupancyGrid>("/map", 10);
    
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
        if (down_map.data.size() != whole_map.data.size()){
            ROS_INFO("DOWN map IS NOT match whole map!!!");
            ROS_INFO("wohle map info ->  size: %zu, width: %d x height: %d", 
            whole_map.data.size(), whole_map.info.width, whole_map.info.height);
            ROS_INFO("down map info ->  size: %zu, width: %d x height: %d", 
            down_map.data.size(), down_map.info.width, down_map.info.height);
            ROS_INFO("up map info ->  size: %zu, width: %d x height: %d", 
            up_map.data.size(), up_map.info.width, up_map.info.height);
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
        }



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

        map_pub.publish(map_out);


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
