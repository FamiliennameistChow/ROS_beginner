#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include <iostream>
#include <vector>  
// #include <map>
// #include <unordered_map>
using namespace std;

typedef vector<pair<geometry_msgs::Point, geometry_msgs::Point> > Point_pair_set;
nav_msgs::OccupancyGrid mapData; 
geometry_msgs::Point x_start, x_end;
visualization_msgs::Marker points, points_goal,line, line_back;
int state=100, out;
// vector<geometry_msgs::Point> rrt_tree;
vector<geometry_msgs::Point> rrt_tree_back; //记录最后的rrt tree
Point_pair_set rrt_tree_set_inv;  //反向rrt tree 储存(当前节，父节点)对
Point_pair_set rrt_tree_set;  //正向rrt tree 储存(当前节，父节点)对


void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    mapData=*msg;
}

void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 

geometry_msgs::Point p;  
p.x=msg->point.x;
p.y=msg->point.y;
p.z=msg->point.z;
points_goal.points.push_back(p);

}

struct CONNECT
{
    bool isConnect;
    bool forward_or_inv;
    geometry_msgs::Point forward_point;
    geometry_msgs::Point inv_point;
};


//　返回两点的欧式距离
float Norm(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return pow( pow(p1.x-p2.x, 2) + pow(p1.y-p2.y,2), 0.5);
}

double getRandData(int min,int max)
{
    double m1=(double)(rand()%101)/101; // 计算 0，1之间的随机小数,得到的值域近似为(0,1)
    min++;  //将 区间变为(min+1,max),
    double m2=(double)((rand()%(max-min+1))+min); //计算 min+1,max 之间的随机整数，得到的值域为[min+1,max]
    m2=m2-1; //令值域为[min,max-1]
    return m1+m2;  //返回值域为(min,max),为所求随机浮点数
}


// 计算rrt_tree上离随机点最近的点
geometry_msgs::Point Near(Point_pair_set &V, geometry_msgs::Point p){
    float min = Norm(V[0].first, p);
    float temp;
    geometry_msgs::Point nearest_point;

    for (auto vp : V)
    {
        temp = Norm(vp.first, p);
        if(temp <=min )
        {
            min = temp;
            nearest_point = vp.first;
        }
    }
    return nearest_point;
}

// 以nearest_point为起点，以nearest_point->rand_point射线为方向，前进eta的距离
geometry_msgs::Point Steer(geometry_msgs::Point nearest_point, geometry_msgs::Point rand_point, float eta_){
    geometry_msgs::Point new_point;
    float dis = Norm(nearest_point, rand_point);
    if(dis <= eta_)
    {
        new_point = rand_point;
    }else{
        new_point.x = nearest_point.x + eta_ * (rand_point.x - nearest_point.x) / dis;
        new_point.y = nearest_point.y + eta_ * (rand_point.y - nearest_point.y) / dis;
    }

    return new_point;
}

// 占据状态　1:占据　0:未占据　-1:未知
int CollisionFree(geometry_msgs::Point nearest_point, geometry_msgs::Point new_point, nav_msgs::OccupancyGrid mapsub){
    std::vector<signed char> Data=mapsub.data;
    float resolution=mapsub.info.resolution;
    float Xstartx=mapsub.info.origin.position.x;
    float Xstarty=mapsub.info.origin.position.y;
    float width=mapsub.info.width;

    float step_length = mapsub.info.resolution * 0.2; //逐步推进的方式检测是否碰撞
    int step_nu = ceil(Norm(nearest_point, new_point)/step_length);  //推进步数
    //ceil(x)返回的是大于x的最小整数
    geometry_msgs::Point step_point = nearest_point;
    
    int state, out;
    for(int i=0; i<step_nu; i++)
    {
        step_point = Steer(step_point, new_point, step_length);
        // floor(x)返回的是小于或等于x的最大整数
        float index=( floor((step_point.y-Xstarty)/resolution)*width)+( floor((step_point.x-Xstartx)/resolution) );
        out = Data[int(index)];
        // ROS_INFO("the value is : %d", out);
        // cout << "out" << out << endl;

        if (out == 100)
        {
            state = 1;
            break;
        }
        if (out == -1)
        {
            state = -1;
            break;
        }
        if (out == 0)
        {
            state = 0;
            continue;
        }
    }

    return state;
}

bool isGoal(geometry_msgs::Point new_point, geometry_msgs::Point end_point){
    if(Norm(new_point, end_point) < 0.5)
    {
        return true;
    }else
    {
        return false;
    }
    
}

CONNECT isConnect(Point_pair_set &V, Point_pair_set &V_INV, int checking_, int checking_inv_, geometry_msgs::Point new_point, geometry_msgs::Point new_point_inv){
    CONNECT Connect_check_;
    Connect_check_.isConnect = false;

    if(checking_ == 0){
        for(auto vp : V_INV){
            if (Norm(vp.first, new_point) < 0.5){
                int check = CollisionFree(vp.first, new_point, mapData);
                if (check == 0){
                    Connect_check_.isConnect = true;
                    Connect_check_.forward_point = new_point;
                    Connect_check_.inv_point = vp.first;
                    break;
                }
            }
        }
    }

    if(checking_inv_ == 0){
        for(auto vp : V){
            if(Norm(vp.first, new_point_inv) < 0.5){
                int check = CollisionFree(vp.first, new_point_inv, mapData);
                if (check == 0){
                    Connect_check_.isConnect = true;
                    Connect_check_.forward_point = vp.first;
                    Connect_check_.inv_point = new_point_inv;
                    break;
                }
            }
        }
    }
    return Connect_check_;

}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "rrt");
    ros::NodeHandle nh;
    ros::Rate rate(10); 

    ros::Subscriber sub= nh.subscribe("/gridMap", 100 ,mapCallBack);	
    ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 100 ,rvizCallBack);
    ros::Publisher point_pub = nh.advertise<visualization_msgs::Marker>("node", 10);
    ros::Publisher line_pub = nh.advertise<visualization_msgs::Marker>("lines", 10);
    ros::Publisher line_back_pub = nh.advertise<visualization_msgs::Marker>("lines_back", 10);	

    // wait until map is received, when a map is received, mapData.header.seq will not be < 1  
    while (mapData.header.seq<1 or mapData.data.size()<1){
        ROS_INFO("waiting for map !");
        ros::spinOnce();  
        ros::Duration(0.1).sleep();
        }

    
    //visualizations points and lines
    points.header.frame_id = mapData.header.frame_id;
    points.header.stamp=ros::Time(0);
    points.ns = "points";
    points.id = 0;
    points.type=points.POINTS;
    points.action=points.ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x=0.05; 
    points.scale.y=0.05;
    points.color.r = 0.0/255.0;
    points.color.g = 0.0/255.0;
    points.color.b = 255.0/255.0;
    points.color.a=1.0;
    points.lifetime = ros::Duration();

    points_goal.header.frame_id = mapData.header.frame_id;
    points_goal.header.stamp=ros::Time(0);
    points_goal.ns = "points_goal";
    points_goal.id = 0;
    points_goal.type=points.POINTS;
    points_goal.action=points.ADD;
    points_goal.pose.orientation.w = 1.0;
    points_goal.scale.x=0.05; 
    points_goal.scale.y=0.05;
    points_goal.color.r = 0.0/255.0;
    points_goal.color.g = 255.0/255.0;
    points_goal.color.b = 255.0/255.0;
    points_goal.color.a=1.0;
    points_goal.lifetime = ros::Duration();

    line.header.frame_id=mapData.header.frame_id;
    line.header.stamp=ros::Time(0);
    line.ns = "lines";
    line.id = 1;
    line.type=line.LINE_LIST;
    line.action=line.ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x =  0.03;
    line.scale.y= 0.03;
    line.color.r =155.0/255.0;
    line.color.g= 155.0/255.0;
    line.color.b =155.0/255.0;
    line.color.a = 1.0;
    line.lifetime = ros::Duration();

    line_back.header.frame_id=mapData.header.frame_id;
    line_back.header.stamp=ros::Time(0);
    line_back.ns = "line_back";
    line_back.id = 1;
    line_back.type=line.LINE_LIST;
    line_back.action=line.ADD;
    line_back.pose.orientation.w = 1.0;
    line_back.scale.x =  0.03;
    line_back.scale.y= 0.03;
    line_back.color.r =255.0/255.0;
    line_back.color.g= 0.0/255.0;
    line_back.color.b =0.0/255.0;
    line_back.color.a = 1.0;
    line_back.lifetime = ros::Duration();

    // 获取起始点与目标点
    while(points_goal.points.size()<3)
    {
        ROS_INFO("set start point and goal in rviz");
        point_pub.publish(points_goal);
        ros::spinOnce();
    }

    ROS_INFO("start rrt!");
    x_start.x = points_goal.points[0].x; x_start.y = points_goal.points[0].y;
    x_end.x = points_goal.points[1].x; x_end.y = points_goal.points[1].y;
    
    rrt_tree_back.clear();
    rrt_tree_set.clear();
    rrt_tree_set_inv.clear();
  
    rrt_tree_set.push_back(make_pair(x_start, x_start));
    rrt_tree_set_inv.push_back(make_pair(x_end, x_end));

    // points.points.clear();
    
    geometry_msgs::Point x_new, x_rand, x_nearest;
    geometry_msgs::Point x_new_inv, x_rand_inv, x_nearest_inv;

    float eta = 0.3;
    int checking, checking_inv;
    CONNECT connect_check;

    int a, nu;
    // int forward_or_inv;
    
    
    while (ros::ok())
    {   
        if(nu % 10 == 0)
        {
            a = cin.get();
            cout << a << endl;
        }

        // forward_or_inv = nu%2; // forward_or_inv 0-forward 1-inv

        ROS_INFO("rand sample a point");
        x_rand.x = getRandData(mapData.info.origin.position.x, mapData.info.origin.position.x+10);
        x_rand.y = getRandData(mapData.info.origin.position.y, mapData.info.origin.position.y+10);
        ROS_INFO("rand point-- x %f, y %f", x_rand.x, x_rand.y);
        points.points.push_back(x_rand);
        point_pub.publish(points);

        x_rand_inv.x = getRandData(mapData.info.origin.position.x, mapData.info.origin.position.x+10);
        x_rand_inv.y = getRandData(mapData.info.origin.position.y, mapData.info.origin.position.y+10);
        ROS_INFO("rand inv point-- x %f, y %f", x_rand_inv.x, x_rand_inv.y);
        points.points.push_back(x_rand_inv);
        point_pub.publish(points);


        x_nearest = Near(rrt_tree_set, x_rand);
        ROS_INFO("nearest point-- x %f, y %f", x_nearest.x, x_nearest.y);
        x_nearest_inv = Near(rrt_tree_set_inv, x_rand_inv);
        ROS_INFO("nearest inv point-- x %f, y %f", x_nearest_inv.x, x_nearest_inv.y);

        x_new = Steer(x_nearest, x_rand, eta);
        ROS_INFO("new point-- x %f, y %f", x_new.x, x_new.y);
        x_new_inv = Steer(x_nearest_inv, x_rand_inv, eta);
        ROS_INFO("new inv point-- x %f, y %f", x_new_inv.x, x_new_inv.y);

        checking = CollisionFree(x_nearest, x_new, mapData);
        checking_inv = CollisionFree(x_nearest_inv, x_new_inv, mapData);

        ROS_INFO("the CollisionFree is : %d", checking);
        // cout << "out: " << out << endl;

        if(checking == 1 ||  checking_inv == 1)
        {
            ROS_INFO("collision");
            // continue;
        }
        if(checking == 0 || checking_inv == 0)
        {
            ROS_INFO("get a new point!");
            // rrt_tree.push_back(x_new);
            if(checking == 0)
            {
                rrt_tree_set.push_back(make_pair(x_new, x_nearest));
                line.points.push_back(x_nearest);
                line.points.push_back(x_new);
                line_pub.publish(line);
            }
            if(checking_inv == 0)
            {
                rrt_tree_set_inv.push_back(make_pair(x_new_inv, x_nearest_inv));
                line.points.push_back(x_nearest_inv);
                line.points.push_back(x_new_inv);
                line_pub.publish(line);
            }

            connect_check.isConnect = false;
            connect_check = isConnect(rrt_tree_set, rrt_tree_set_inv, checking, checking_inv, x_new, x_new_inv);
            if(connect_check.isConnect == true)
            {   
                points_goal.points.push_back(connect_check.inv_point);
                points_goal.points.push_back(connect_check.forward_point);
                point_pub.publish(points_goal);

                rrt_tree_back.clear();
                rrt_tree_set.push_back(make_pair(connect_check.inv_point, connect_check.forward_point));
                rrt_tree_back.push_back(connect_check.inv_point); //回溯rrt
                geometry_msgs::Point back_point;
                geometry_msgs::Point p;

                back_point = connect_check.inv_point;
                // back tree
                int num = rrt_tree_set.size();
                for(int i=0; i<num; i++)
                {
                    for(auto iter=rrt_tree_set.begin(); iter!=rrt_tree_set.end(); iter++){
                        if(Norm((*iter).first, back_point) == 0)
                        {
                            back_point = (*iter).second;
                            // geometry_msgs::Point p;
                            p.x = (*iter).first.x;
                            p.y = (*iter).first.y;
                            p.z = (*iter).first.z;
                            line_back.points.push_back(p);
                            p.x = (*iter).second.x;
                            p.y = (*iter).second.y;
                            p.z = (*iter).second.z;
                            line_back.points.push_back(p);
                            line_back_pub.publish(line_back);
                            break;
                        }
                    }
                    rrt_tree_back.push_back(back_point);
                    cout<<"dis from start: " << Norm(back_point, x_start) << endl;
                    if(Norm(back_point, x_start) <= 0.05)
                    {   
                        ROS_INFO("back tree sucess!");
                        break;
                    }
                }

                // back inv tree
                back_point = connect_check.inv_point;
                int num_inv = rrt_tree_set_inv.size();
                for(int i=0; i<num_inv; i++)
                {
                    for(auto iter=rrt_tree_set_inv.begin(); iter!=rrt_tree_set_inv.end(); iter++){
                        if(Norm((*iter).first, back_point) == 0)
                        {
                            back_point = (*iter).second;
                            // geometry_msgs::Point p;
                            p.x = (*iter).first.x;
                            p.y = (*iter).first.y;
                            p.z = (*iter).first.z;
                            line_back.points.push_back(p);
                            p.x = (*iter).second.x;
                            p.y = (*iter).second.y;
                            p.z = (*iter).second.z;
                            line_back.points.push_back(p);
                            line_back_pub.publish(line_back);
                            break;
                        }
                    }
                    rrt_tree_back.push_back(back_point);
                    cout<<"dis from end: " << Norm(back_point, x_end) << endl;
                    if(Norm(back_point, x_end) <= 0.05)
                    {   
                        ROS_INFO("back inv tree sucess!");
                        break;
                    }
                }

                // cout<< "rrt num: " << rrt_tree.size() << endl;
                cout<< "back num: " << rrt_tree_back.size() << endl;
                ROS_INFO("you have got the goal!----");
                break;
            }
            // point_pub.publish(points);
            // points.points.clear();
        }
        if(checking == -1 || checking_inv == -1)
        {
            ROS_INFO("get a unknown area!");
            // continue;
        }

        nu++;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}