/*
* @file  : pose_yaw_control.h
* @brief : class for yaw control
* @date  : 2019.01.09
* @author: kumaron
* 
*/
#include "pose_yaw_control.h"

void quaterDataAssign(Quaternion* set_yaw_data, float set_yaw_x, float set_yaw_y, float set_yaw_z, float set_yaw_w, int set_yaw_theta)
{
    set_yaw_data->x = set_yaw_x;
    set_yaw_data->y = set_yaw_y;
    set_yaw_data->z = set_yaw_z;
    set_yaw_data->w = set_yaw_w;
    set_yaw_data->theta = set_yaw_theta;
}
