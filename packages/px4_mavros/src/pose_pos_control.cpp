/*
* @file  : pose_pos_control.cpp
* @brief : class for position control
* @date  : 2019.01.09
* @author: kumaron
* 
*/
#include "pose_pos_control.h"

float pos_x_step = 0.1;
float pos_y_step = 0.1;
float pos_z_step = 0.1;


PositionData* PositionControl::positionControl(int control_axis_type, bool control_direction_type, float control_theta)
{
        float *temp_x = &(position_data->x);
        float *temp_y = &(position_data->y);
        float *temp_z = &(position_data->z);
        float step = 0;
        float theta_sin = sin(control_theta*M_PI/180);
        float theta_cos = cos(control_theta*M_PI/180);
        switch (control_axis_type)
        {
            case M_X_AXIS:
                step = pos_x_step;
                if (control_direction_type){
                    (*temp_x) += theta_cos*step;
                    (*temp_y) += theta_sin*step;
                }
                else{
                    (*temp_x) -= theta_cos*step;
                    (*temp_y) -= theta_sin*step;
                }
                break;
            case M_Y_AXIS:
                step = pos_y_step;
                if (control_direction_type){
                    (*temp_x) += -theta_sin*step;
                    (*temp_y) += theta_cos*step;  
                }
                else{
                    (*temp_x) -= -theta_sin*step;
                    (*temp_y) -= theta_cos*step;  
                }              
                break;
            case M_Z_AXIS:
                step = pos_z_step;
                if (control_direction_type){
                    (*temp_z) += step;
                }
                else{
                    (*temp_z) -= step;
                }                
                break;
            default:
                break;
        }
        if(position_data->z < 0){
            position_data->z = 0;
        }
        return position_data;
}

void positionValueAssign(PositionData *set_pos_data, float set_pos_x, float set_pos_y, float set_pos_z){
    set_pos_data->x = set_pos_x;
    set_pos_data->y = set_pos_y;
    set_pos_data->z = set_pos_z;
}

