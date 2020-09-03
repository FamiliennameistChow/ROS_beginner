#ifndef _DATA_TYPE_
#define _DATA_TYPE_

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>

#define inf 1>>30
using namespace std;

struct Cube;

struct Cube
{     
      //Eigen::Vector3d p0, p1, p2, p3, p4, p5, p6, p7;   // the 8 vertex of a cube 
      Eigen::MatrixXd vertex;
      Eigen::Vector3d center; // the center of the cube
      bool valid;    // indicates whether this cube should be deleted

      double t; // time allocated to this cube
      std::vector< std::pair<double, double> > box;
/*
           P3------------P2 
           /|           /|              ^
          / |          / |              | z
        P0--|---------P1 |              |
         |  P7--------|--p6             |
         | /          | /               /--------> y
         |/           |/               /  
        P4------------P5              / x
*/                                                                                 

      // create a cube using 8 vertex and the center point
      Cube( Eigen::MatrixXd vertex_, Eigen::Vector3d center_)
      {
            vertex = vertex_;
            center = center_;
            valid = true;
            t = 0.0;
            box.resize(3);
      }

      // create a inscribe cube of a ball using the center point and the radius of the ball
      void setVertex( Eigen::MatrixXd vertex_, double resolution_)
      {     
            vertex = vertex_;
            vertex(0,1) -= resolution_ / 2.0;
            vertex(3,1) -= resolution_ / 2.0;
            vertex(4,1) -= resolution_ / 2.0;
            vertex(7,1) -= resolution_ / 2.0;

            vertex(1,1) += resolution_ / 2.0;
            vertex(2,1) += resolution_ / 2.0;
            vertex(5,1) += resolution_ / 2.0;
            vertex(6,1) += resolution_ / 2.0;

            vertex(0,0) += resolution_ / 2.0;
            vertex(1,0) += resolution_ / 2.0;
            vertex(4,0) += resolution_ / 2.0;
            vertex(5,0) += resolution_ / 2.0;

            vertex(2,0) -= resolution_ / 2.0;
            vertex(3,0) -= resolution_ / 2.0;
            vertex(6,0) -= resolution_ / 2.0;
            vertex(7,0) -= resolution_ / 2.0;

            vertex(0,2) += resolution_ / 2.0;
            vertex(1,2) += resolution_ / 2.0;
            vertex(2,2) += resolution_ / 2.0;
            vertex(3,2) += resolution_ / 2.0;

            vertex(4,2) -= resolution_ / 2.0;
            vertex(5,2) -= resolution_ / 2.0;
            vertex(6,2) -= resolution_ / 2.0;
            vertex(7,2) -= resolution_ / 2.0;
            
            setBox();
      }
      
      void setBox()
      {
            box.clear();
            box.resize(3);
            box[0] = std::make_pair( vertex(3, 0), vertex(0, 0) ); // x 方向长度
            box[1] = std::make_pair( vertex(0, 1), vertex(1, 1) ); // y 方向长度
            box[2] = std::make_pair( vertex(4, 2), vertex(1, 2) ); // z 方向长度
      }

      void printBox()
      {
            std::cout<<"center of the cube: \n"<<center<<std::endl;
            std::cout<<"vertex of the cube: \n"<<vertex<<std::endl;
      }

      Cube()
      {  
         center = Eigen::VectorXd::Zero(3);
         vertex = Eigen::MatrixXd::Zero(8, 3);

         valid = true;
         t = 0.0;
         box.resize(3);
      }

      ~Cube(){}
};

namespace std{
    template<>
    struct hash<octomap::point3d>{//哈希的模板定制
    public:
        size_t operator()(const octomap::point3d &pt) const 
        {
            return hash<double>()(pt(0)) ^ hash<double>()(pt(1)) ^ hash<double>()(pt(2));
        }
        
    };
    
    template<>
    struct equal_to<octomap::point3d>{//等比的模板定制
    public:
        bool operator()(const octomap::point3d &p1, const octomap::point3d &p2) const
        {
            return p1(0) == p2(0) && p1(1) == p2(1) && p1(2) == p2(2);
        }
        
    };
}

#endif